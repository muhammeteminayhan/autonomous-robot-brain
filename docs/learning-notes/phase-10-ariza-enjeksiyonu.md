# Phase 10 — Arıza Enjeksiyonu: Filtre bozulunca ne yapar?

## 1. Arıza NEREYE enjekte edilir — ve bu neden her şeydir?

```
Gazebo  --ham-->  arb_sensors  --işlenmiş-->  EKF / haritalama / planlama
                       ^
                       |  fault.* parametreleri, runtime'da
                  fault_scheduler
```

Arızalar **sensör ön-işleme katmanına** enjekte edilir, simülatöre değil.

Bunun sonucu: tekerlekler her senaryoda **aynı** komutları alır, simülatör
arızadan habersizdir, dolayısıyla **gerçek yörünge altı senaryoda da
birebir aynıdır**. Case A ile Case C arasındaki fark **sadece kestirim
hatasıdır**, başka hiçbir şey değil.

Gazebo'nun içine enjekte etseydik fizik değişirdi, robot fiziksel olarak başka
bir yere giderdi ve ortaya çıkan hata "kötü kestirim" ile "farklı yörünge"nin
karışımı olurdu — ve bunları sonradan **ayıramazdık**.

Aynı mantık **açık çevrim rota**yı zorunlu kılar. Robot zamanlanmış hız
komutlarıyla sürülür, bir hedefe doğru değil. Kapalı çevrim olsaydı kontrolcü
**bozuk tahmine göre** direksiyon kırardı ve karşılaştırmanın dayandığı özellik
yok olurdu.

## 2. Üç parça, ve neden ayrıldılar

| Parça | Nerede | Ne |
|---|---|---|
| Knob zaman çizelgesi | `arb_evaluation/scenario.py` | **Saf fonksiyon**: (senaryo, t) → knob'lar. ROS yok. |
| Zamanlayıcı | `arb_evaluation/fault_scheduler.py` | Saati okur, parametre servisi çağırır, `FaultState` yayınlar. |
| Koşucu + puanlayıcı | `scripts/run_scenario.py` | Rotayı sürer, kaydeder, puanlar. |

`knobs_at(scenario, t)`'nin **saf** olması, zaman çizelgesinin test edilebilir
olmasının **tek** sebebi. Çalışan bir simülatöre karşı test edilse, **yanlış bir
rampa** ile **yavaş bir parametre servisi** birbirinin tıpatıp aynısı görünür.

56 unit test + 13 enjekte mutasyon, hepsi yakalandı.

## 3. Sim time, wall time değil

Gazebo'nun `real_time_factor`'ı sınırsız — bu dünya 1.0'ın epey üstünde koşuyor.
Wall time'a göre çalışan bir zamanlayıcı, "t = 20 s"te tanımlı bir arızayı
**rastgele bir simüle anda** uygulardı, robot her koşuda başka bir yerde olurdu
ve **Case A hiçbir şeyin baseline'ı olmazdı**.

## 4. Neden polling, neden önceden hesaplanmış zamanlama değil?

Cazip ve yanlış: encoder bias'ı 5 saniye boyunca **sürekli** rampalanıyor.
Yani doğru değerin değiştiği **sonlu bir an kümesi yok**. Sabit hızda
yoklamak, rampanın çözünürlüğünü (`rate_hz`) **açıkça** belirtir, gizlemez.

## 5. Kompozisyon — dosya sırası fiziksel bir özellik değildir

Arızalar **üst üste biner** — Case F'in bütün amacı bu. İki arıza aynı knob'a
dokunuyorsa **güçlü olan kazanır**: çarpan büyük faktörü, bias büyük genliği
alır, outage her şeyi ezer.

Sonraki arızanın öncekini **ezmesi** yanlış olurdu: YAML'daki sıra fiziksel bir
gerçek değil, robotun ne yaşayacağına karar vermemeli.

## 6. Case C neden en sinsi olan?

Tek tekerlekte sabit bias, **sadece encoder'lara bakarak gerçek bir dönüşten
ayırt edilemez.**

```
Sol teker 2.00 rad/s, sağ teker 2.05 rad/s
  -> encoder'lar: "sağa dönüyoruz"  (tutarlı, artık yok, çelişki yok)
  -> gerçek:      "düz gidiyoruz"
```

Dead reckoning sessizce ve **tutarlı biçimde** kıvrılır. Tespit edilecek hiçbir
şey yok: residual yok, çelişki yok.

Yakalayabilen tek şey **bağımsız bir yön kaynağı** (jiroskop) ya da **mutlak
bir kaynak** (LiDAR scan matching). EKF'in ikisini birden füzyonlamasının sebebi
tam olarak bu.

### Rampa neden var?

Bias **5 saniyede lineer** artıyor, basamak değil. Basamak tespiti kolaydır ve
donanım öyle bozulmaz: gevşeyen bir teker bağlantısı ya da ısınan bir MEMS
elemanı saniyeler içinde bozulur. **Bariz biçimde bir şeyin değiştiği tek bir an
yoktur.**

## 7. Ölçtüğümüz üç şey

### Position / Heading RMSE — üçe bölünmüş

Sağlıklı faz (ilk arızadan önce) / bozuk faz / toparlanma fazı.

Üçte biri sağlıklı geçmiş bir koşunun **tek** bir RMSE'si, iki yarının **hiçbiri**
hakkında bir şey söylemez.

### Recovery time — tanımı önemli

Son arıza kalktıktan sonra, pozisyon hatasının **aynı koşunun sağlıklı fazında
tuttuğu bandın** içine dönmesi ve orada **3 saniye kalması** için geçen süre.

Bandı **kendi koşusunun** sağlıklı fazından alıyoruz, Case A'dan değil: iki koşu
arızayla ilgisi olmayan sebeplerle (fizik seed'i, haritanın nerede kurulduğu)
gürültü tabanından fazla ayrışır, ve başka bir koşudan alınan eşik "recovery
time"ı o farkın **artefaktı** yapardı.

Sonuç sadece sayı değil, **sonuç etiketi** ile raporlanır:
`no fault` / `no post-fault window` / `never left the band` / `recovered` /
`did not recover`.

> "Arıza yoktu" ile "anında toparladı"nın ikisini de **0.0** diye raporlamak
> onları ayırt edilemez hale getirirdi.

### Covariance consistency — filtre yalan söylüyor mu?

Hatanın, filtrenin **kendi** raporladığı 2σ'nın içinde kaldığı örnek oranı.

**Bozulan bir filtre ile yalan söyleyen bir filtre arasındaki fark budur.**
Hatası büyürken kovaryansı dar kalan bir filtre, kötü bir filtreden **daha
kötüdür**: aşağı akıştaki her şey ona inanır.

## 8. Her arıza senaryo bitmeden kalkar

Her arıza, koşu bitmeden **25 s önce** sona erer.

İlk versiyonda her arıza son saniyeye kadar sürüyordu — bu da **recovery
time**'ı, yani bu fazın raporlamak için var olduğu üç sayıdan birini, **altı
vakanın dördünde ölçülemez** yapıyordu: sensörlerin tekrar sağlıklı olduğu bir
aralık hiç yoktu.

`test_every_scenario_with_a_fault_leaves_a_recovery_window` artık bunu zorluyor.

## 9. Config'in iddia ettiğini yapamayan bir knob

`configs/scenarios/*.yaml` her IMU arızası için `affect_accelerometer` beyan
ediyordu: Case B tüm paketi bozar, Case F sadece jiroyu.

Ama ön-işlemcide **tek** bir `fault.noise_scale` vardı ve **iki kanalı birden**
ölçekliyordu. Yani o ayar bir **ayar değil, yorumdu** — Case F ivmeölçeri de
bozuyordu ve bunu kimse söylemiyordu.

Artık ayrı bir `fault.accel_noise_scale` var. Poz kestirimini **değiştiremez**,
ve asıl nokta bu: **EKF ivmeölçeri hiç füzyonlamıyor.** Düzlemsel bir robot için
yürüme hızında faydalı sinyal, milideredelik şasi eğiminden gelen yerçekimi
sızıntısının altında gömülü; iki kez integre edip pozisyon elde etmek tekerlek
encoder'larından kesinlikle daha kötü.

Case B bu iddiayı **ölçüyor**, sadece söylemiyor.

> **Ders:** sessizce yok sayılan bir config değeri, hiç olmayan bir config
> değerinden **daha kötüdür**.

## 10. Bu fazda tekrar ısırılan bir tuzak

```python
self._clients = {...}     # YANLIŞ
self._param_clients = {}  # doğru
```

`rclpy.Node` kendi servis istemci listesini `self._clients`'te tutar. Üzerine
dict atayınca executor **string'ler üzerinde** dönmeye çalışır:

```
AttributeError: 'str' object has no attribute '_executor_event'
KeyError: 0                      # destroy_node() bir dict'i [0] ile indeksliyor
```

Traceback tamamen rclpy iç dosyalarını gösterir, **senin dosyanı hiç anmaz**.
Bu projede **ikinci kez** oldu (ilki Phase 2'de `arb_sensors`'ta).

Bir de bunun yanında: **rclpy'nin logger'ı printf-style format almaz** —
`rclcpp` alır. `logger.info("x=%d", 5)` doğrudan `TypeError` fırlatır.
f-string kullan.

## Sonuçlar

| Vaka | Senaryo | Pozisyon RMSE [m] | A'ya göre | Yön RMSE [deg] | A'ya göre | Bozuk faz poz. RMSE [m] | Toparlanma [s] | 2-sigma içinde |
|---|---|---|---|---|---|---|---|---|
| A | Nominal | 0.0452 | 1.00x | 2.97 | 1.00x | -- | n/a | 76 % |
| B | IMU noise x10 | 0.0621 | 1.37x | 4.30 | 1.45x | 0.0535 | 14.9 | 65 % |
| C | Encoder drift | 0.1646 | 3.64x | 12.54 | 4.22x | 0.1162 | **asla** | 47 % |
| D | LiDAR dropout 60 % | 0.0352 | 0.78x | 2.60 | 0.88x | 0.0332 | hiç sapmadı | 92 % |
| E | Sensor outage 10 s | 0.0578 | 1.28x | 3.20 | 1.08x | 0.0647 | 0.9 | 56 % |
| F | Multi-degradation | 0.0644 | 1.43x | 4.38 | 1.47x | 0.0544 | 1.4 | 70 % |

Tekerlek odometrisi **kontrol grubu** olarak yanında koşuyor: B, D ve E
vakalarında onun için sonuç nominalin 0.98-1.00 katı — çünkü **hiçbiri bir
tekerlek encoder'ına dokunmuyor.** Ayrım fazının "hasar" değil "sürüklenme"
okuduğunu böyle anlıyoruz.

### Nasıl okunur?

**EKF her vakada dead reckoning'i yeniyor**, pozisyonda 1.60x - 2.77x.
Füzyon yapmanın bütün gerekçesi bu, ölçülmüş hali.

**Arıza türleri teorinin öngördüğü sırada.** Encoder bias'ı (3.6x) jiro
gürültüsünden (1.4x) çok daha kötü, o da tam sensör kesintisinden (1.3x) kötü,
o da LiDAR ışınlarının %60'ını kaybetmekten (ölçülebilir etki yok) kötü.

> Bir arızayı tehlikeli yapan şey **ne kadar sinyal yok ettiği değil**, onu
> **başka bir şeyin yalanlayıp yalanlayamadığıdır.**

**Case C yığını yeniyor** — ve buradaki en faydalı sonuç bu. Filtre yönünü
**±0.31 derece** güvenle raporluyor, ama **12 derece** yanlış. Scan matcher
253 eşleşmenin 253'ünü 0.96 skorla kabul ediyor.

Harita **EKF'in kendi tahmin ettiği pozda** kuruluyor. Tahmin sürüklendikçe
harita da onunla sürükleniyor, ve sonra scan matching sürüklenmiş pozu
**onaylıyor**. Döngüde hiçbir dış referans yok.

**Filtre fazla kendine güveniyor**: vakaya göre kendi 2σ'sı hakkında zamanın
%8 ila %53'ünde yanılıyor.

### Case F neden "birikmedi"?

Case F üç arızayı aynı anda koşuyor ve **1.43x** çıkıyor — tek arızalı Case
C'nin **3.64x**'inden **daha hafif**. Üstelik **1.4 s'de toparlıyor**, Case C
ise hiç toparlamıyor.

Sebep birikme değil: sonucu **tek en güçlü bileşen** belirliyor, ve Case F'in
encoder bias'ı daha zayıf (0.04 vs 0.05 rad/s).

Bu, kendine-referanslı haritada bir **eşik** olduğunu düşündürüyor: belli bir
birikmiş sürüklenmenin altında harita toparlanmaya yetecek kadar iyi kalıyor,
üstünde ise harita hatanın etrafında yeniden çizilmiş oluyor.

> **Sonuç değil, hipotez.** C ile F arasında iki değişken birden farklı ve her
> ikisi de **tek koşu**. Phase 11'in yapması gereken deney tam olarak bu: tek
> arıza, genliği taranmış, tohumlar üzerinden tekrarlanmış.

Tam yazım: [`docs/fault-injection.md`](../fault-injection.md).

## Mini görev

> 1. `case_c_encoder_drift.yaml`'da `ramp_time_s`'i 5.0 → 0.0 yap (basamak
>    arıza) ve senaryoyu tekrar koştur. EKF'in `odom ... (N gated)` sayacı ne
>    oluyor? **Neden** rampa basamaktan daha sinsi?
> 2. `odometry.launch.py`'ı `scan_match:=false` ile başlatıp Case C'yi koştur.
>    Position RMSE kaça çıkıyor? Bu sana jiro'nun tek başına ne kadar
>    koruduğunu söylüyor mu?
> 3. Case E'de IMU outage'ı 10 s → 40 s yap. EKF **toparlıyor mu**? Kovaryans
>    ne yapıyor, ve `fraction_inside_2sigma_pos` ne oluyor?
