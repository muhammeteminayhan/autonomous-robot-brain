# Phase 6 — Occupancy Grid ve LiDAR Tabanlı Mutlak Düzeltme

---

## 1. Log-odds — neden olasılıkla değil?

İstediğimiz şey hücre başına `P(dolu | tüm ölçümler)`. Bayes'i doğrudan
uygularsak her ölçümde normalleştirme paydası yeniden hesaplanmalı; aritmetik
0 veya 1'e yakın sayıların çarpımı olur ve **alt taşar**.

Log-odds tanımla:

```
l(m) = log( P(m) / (1 − P(m)) )
```

Bağımsızlık varsayımı altında Bayes **toplamaya** dönüşür:

```
l_t = l_{t−1} + l_sensör(m | z_t) − l_0
```

`l_0` önsel'in log-odds'u (0.5 için sıfır). Güncelleme **bir toplama ve bir
clamp**. Alt taşamaz, simetriktir (lehte ve aleyhte kanıt tam olarak
birbirini götürür) ve bu simetri haritanın **fikrini değiştirebilmesini**
sağlar.

Geri dönüş: `p = 1 − 1/(1 + e^l)`.

### `l_0` neden çıkarılıyor?

Önsel 0.5 ise `l_0 = 0` ve çıkarma hiçbir şey yapmaz — bu yüzden hata
görünmez. Ama önsel farklıysa (kapalı mekân haritası "muhtemelen boş" diye
başlayabilir), çıkarmayı atlamak **önseli teyit eden bir ölçümün bile** hücreyi
önselden uzaklaştırmasına yol açar. Hücreler sırf bakıldıkları için kayar.

---

## 2. Ters sensör modeli

"Ters", çünkü `P(harita | ölçüm)` sorusuna cevap verir, `P(ölçüm | harita)`'ya
değil. Menzil ölçer için kasten **kaba**:

- Işının **geçtiği** her hücre → boş lehine kanıt (`p_miss`)
- Işının **bittiği** hücre → dolu lehine kanıt (`p_hit`)
- Diğer her hücre → dokunulmaz

Kabalık kasıtlı. Saniyede yüzlerce ışın entegre edilirken fiziksel olarak
ayrıntılı bir model çok az şey kazandırır ve şemayı işe yarar kılan tek
özelliği — güncellemenin **hücre başına sabit bir toplama** olması — kaybeder.

### `p_hit > 1 − p_miss` neden?

`p_hit = 0.75`, `p_miss = 0.35`. Simetrik değil, ve bu kasıtlı: bir yerde
**duran** ışın o tek hücre hakkında, geçip giden bir ışının yolundaki her bir
hücre hakkında söylediğinden **daha fazlasını** söyler.

---

## 3. Clamping — neden zorunlu?

Clamp olmazsa 500 kez görülen bir duvar `+250` log-odds'a ulaşır. O duvarda bir
kapı açılsa, kaydedilmesi için **500 çelişen gözlem** gerekir.

Clamping, bir hücrenin biriktirebileceği kanıtı sınırlar ve dolayısıyla
**unutma süresini** sınırlar. `[−2.0, +3.5]` ile doymuş bir hücre ~10 gözlemde
dönüyor. Test: `ClampingBoundsHowLongACellTakesToChangeItsMind`.

---

## 4. Bresenham — neden alt-adımlı ışın yürüyüşü değil?

Işın boyunca küçük adımlarla ilerlemenin iki başarısızlık modu var:

1. **Aynı hücreyi iki kez ziyaret etmek** → o hücrenin kanıtı iki kat sayılır
2. **Bir hücreyi atlamak** → çaprazda delik kalır, planlayıcı duvarın içinden
   yol bulur

Bresenham tam sayı aritmetiğidir: her hücreyi **tam bir kez** ziyaret eder.

### Uç nokta neden hariç?

Uç nokta ışının **çarptığı** hücredir. Önce boş işaretleyip sonra dolu
işaretlemek iki güncellemeyi birbiriyle kavga ettirir.

### Yön bağımsızlığı

Düz Bresenham tam çapraz beraberlikleri **başladığı uca göre** kırar. Aynı
duvar soldan ve sağdan tarandığında farklı hücreler temizlenir ve harita
çaprazlarda **yön-bağımlı dikişler** geliştirir.

Çözüm: uçları sıralayıp geçişi kanonikleştir, sonra çağıranın yönüne çevir.
Maliyet: bir karşılaştırma.

---

## 5. Max-range ışını — atmak da yanlış, işaretlemek de

Hiçbir şeye çarpmayan bir ışın **gerçek bilgi taşır**: o yol boştu.

- **Atarsan** → açık zemin kalıcı olarak `unknown` kalır, planlayıcı geçemez
- **Uç noktasını dolu işaretlersen** → sensörün menzil sınırında, robotun
  etrafında **kusursuz bir yay** şeklinde hayalet duvar çizersin

Doğrusu: ışın geçtiği hücreleri temizler, uç nokta işaretlenmez.

---

## 6. UNKNOWN, FREE'nin bir tonu değildir

Üç durum gerçekten farklı:

- `unknown` yerine `free` sayan bir planlayıcı, hiç görmediği bir duvarın
  içinden kendinden emin geçer
- `occupied` sayan bir planlayıcı keşfetmeyi reddeder

`nav_msgs/OccupancyGrid` bunu `-1` ile ifade eder. `50` yayınlamak her
tüketiciye "bu hücre %50 dolu" der — planlayıcı bunu "geçilebilir ama riskli"
diye okur, "hiç gözlenmemiş" diye değil.

---

## 7. BULUNAN BUG #1 — `map` çerçevesinin mutlak çapası yok

İlk harita skorları: **precision 0.308**.

Sebep: EKF ilk odometri mesajından başlatılıyordu, o da `(0,0)`. Robot ise
`(−8.5, 0)`'da doğuyor. Ölçülen fark: **+8.48 m**. Harita mükemmel
oluşturulmuştu, sadece tamamen kaydırılmıştı.

**Hiçbir şey robota dünyanın orijininin nerede olduğunu söylemiyor.** Tekerlek
encoder'ı ve jiroskop yalnızca **değişimi** ölçer.

Gerçek robotlar aynı şekilde çözer: **söylersin**. AMCL başlangıç poz tahmini
alır, şarj istasyonu bir tane tanımlar, fiducial marker bir tane sağlar. Bu bir
çalışma-zamanı poz ölçümü değildir ve ground truth değildir; `map` çerçevesini
anlamlı kılan **çapa**dır.

Düzeltme sonrası: **0.308 → 0.756**.

---

## 8. BULUNAN BUG #2 — Bayat poz, ışınsal bulanıklık

Harita, en yeni `/arb/pose` mesajını alıyordu. Ama EKF 50 Hz yayınlıyor ve
tahmini ölçüm tamponu kadar gecikiyor, yani en yeni poz taramadan **50-80 ms**
eski olabiliyor.

0.7 rad/s dönerken ve 0.3 s tolerans ile bu **12 derecelik** bir hata; 8 m'de
ışını **1.7 m** yana süpürür. Görünen belirti: duvarların **içinden** geçen
serbest ışınlardan oluşan bir **yıldız patlaması**.

Çözüm: pozu TF'ten, **taramanın zaman damgasında** al. `tf2` kuşatan
dönüşümler arasında interpolasyon yapar — zaten bunun için var. Bonus:
`map → lidar_link` doğrudan geliyor, yani sensör montaj ofseti bu node'un
bilmesi gereken bir sayı olmaktan çıkıp URDF'te kalıyor.

---

## 9. BULUNAN BUG #3 — Kâşif tek odada sıkıştı

Saf tepkisel "engelden kaç" kâşifi başladığı odada bir döngüye yerleşti:
**54 m sürüş, %29.5 hücre gözlemlendi**.

**Duvar takibi** rastgele gezginciyi gerçekten kat planını kapsayan bir şeye
çevirir: bir duvarı sabit mesafede tutmak her bağlı bölgenin sınırını çizer ve
bir kapıda duvarı takip etmek robotu içeri sokar.

Sonuç: **82 m sürüş, %65.7 hücre gözlemlendi**.

---

## 10. BULUNAN BUG #4 — Scan matching hiç füze edilmiyordu

Harita artık yapısal olarak doğruydu ama birkaç derece **döndürülmüştü** — 82 m
boyunca biriken heading drift'i, gidecek yeri yoktu.

LiDAR scan matching eklendi. Ama EKF raporu şunu diyordu:

```
scan-match 0/611 fused | out-of-order 611
```

**611 düzeltmenin hepsi reddedilmişti.** Scan matcher bir taramayı beklemek,
TF sorgulamak ve kaba-ince arama yapmak zorunda, bu yüzden sonucu yayınlandığı
anda birkaç yüz milisaniye eski. Tampon penceresi 60 ms idi.

İki gereksinim çatışıyordu:

- **Füzyon** zaman damgası sırasında olmalı → tampon en yavaş sensörü kapsamalı
- **Kontrol** güncel bir kestirim ister → 350 ms bayat kestirim, 0.35 m/s'de
  12 cm hata demek

Çözüm: ikisini ayır. Filtre sırayla füze eder, **yayın ise filtrenin bir
kopyasını şu ana kadar ileri tahmin ederek** yapılır. Tahmin belirsizliği
artırır — boşluğu gizlemek yerine, kovaryansın tam da bunun için var olması.

Düzeltme sonrası: `scan-match 30/30 fused, out-of-order 0`, ve raporlanan
belirsizlik **1.003 m → 0.017 m**, heading **4.10° → 0.33°**.

---

## 11. Likelihood field — neden ikili değil?

Bir aday pozu "her ışın ucundaki hücre dolu mu?" diye puanlamak neredeyse her
yerde **düz** bir yüzey verir ve hücre sınırlarında sıçrar. Takip edilecek
gradyan yok, ve 2 cm sapmış bir pozu 2 m sapmıştan ayırmanın yolu yok: ikisi de
sıfır alır.

Likelihood field ikili soruyu düzgün bir soruyla değiştirir: en yakın engel ne
kadar **uzakta**?

```
skor = exp(−d² / 2σ²)
```

Yüzey sürekli, doğru pozda tepe yapıyor ve komşuluğunda monoton — herhangi bir
aramanın çalışmasını sağlayan tam da bu.

### Bilineer interpolasyon neden gerekli?

En yakın hücreyi örneklemek skor yüzeyini hücre genişliğinde basamaklara
çevirir; en iyi poz ancak yarım hücre (2.5 cm) doğrulukta bulunur. **Ölçüldü:**
zaten tam hizalı bir tarama 4.2 cm "düzeltiliyordu". İnterpolasyon yüzeyi
sürekli yapar; maliyeti nokta başına üç çarpma.

### Neden Gauss-Newton veya ICP değil?

Gradyan yöntemleri başladıkları havzaya yakınsar. Bir **koridorda** koridor
ekseni boyunca havza neredeyse düzdür, bu yüzden kayarlar. Sınırlı bir pencere
üzerinde **tüketici arama** ya o pencerede global optimumu bulur ya da düşük
skor raporlar — **kendinden emin şekilde yanlış cevap verdiği bir mod yoktur**.

Kaba-sonra-ince: 1 cm'de ±0.4 m ve ±10° tek geçiş 256.000 aday; kaba+ince aynı
erişim ve çözünürlüğe ~3.000 adayda ulaşır.

---

## 12. Kendine gönderme problemi (dürüstlük notu)

Harita, scan matcher'ın düzelttiği kestirimden inşa ediliyor. Yani **mükemmel
bir eşleşme, taramanın robotun kendi inançlarıyla uyuştuğunu kanıtlar, dünyayla
değil.**

Bu yüzden raporlanan kovaryansın bir **tabanı** var: kanıt gerçek ama bağımsız
değil, ve aksi söylenen bir filtre belirsizliğini kendi fikri üzerinden sıfıra
indirir.

Döngüyü düzgün kırmak **loop closure** ve **pose-graph optimizasyonu**
gerektirir — bu projenin kapsamı dışında ve limitations bölümünde adı geçiyor.

---

## 13. Ölçülen sonuçlar

| Aşama | Precision | Recall | F1 | Keşfedilen |
|---|---|---|---|---|
| Çapasız `map` çerçevesi | 0.308 | 0.258 | 0.281 | %30.7 |
| Çapa + TF interpolasyonu + duvar takibi | 0.277 | 0.273 | 0.275 | **%65.7** |
| + scan matching (füzyon bozuk) | 0.527 | 0.487 | 0.507 | %62.6 |
| + scan matching (füzyon çalışıyor) | 0.503 | 0.436 | 0.467 | %65.1 |

Serbest alan doğruluğu her durumda **%94-96**.

Tolerans **bir hücre** (5 cm) — sıkı bir sınır. Duvar sınırı gerçekten bir
hücrenin içine düşer ve harita bir rasterdir, yani daha sıkı bir eşleşme
istemek rasterin yuvarlamasına fit etmek olur.

**Nitel kazanım sayısal olandan daha önemli:** scan matching olmadan harita
birkaç derece **dönüyordu**; onunla dış duvarlar ve oda bölmeleri doğru yerde.

---

## Mini görev

> `ros2 launch arb_localization odometry.launch.py scan_match:=false` ile
> stack'i başlat, `scripts/evaluate_mapping.py --duration 200 --tag nomatch`
> çalıştır.
>
> 1. `results/phase6/map_nomatch.png` ile `map_scanmatch.png` arasındaki
>    **yapısal** fark ne?
> 2. EKF'in raporladığı `1-sigma pos` ve `heading` nasıl değişiyor?
> 3. Scan matcher'ın `min_map_obstacles` parametresini 400'den 50'ye düşürsen
>    ne olur? **Neden tehlikeli?**
