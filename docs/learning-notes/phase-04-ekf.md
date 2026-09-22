# Phase 4 & 5 — Extended Kalman Filter ve Lokalizasyon Değerlendirmesi

Phase 3 şunu ölçmüştü: 10 m'lik bir rotada odometri **%0.94** hata yapıyor ve
tüm hata **dönüşlerde** birikiyor. Phase 4'ün tek amacı o hatayı bağımsız bir
yön kaynağıyla sınırlamak.

**Sonuç:** pozisyon RMSE **%68**, heading RMSE **%60.6** azaldı.

---

## 1. Durum vektörü seçimi — neden bu altı?

```
x = [ x, y, θ, v, ω, b_gyro ]ᵀ
```

| Durum | Neden var |
|---|---|
| `x, y, θ` | Aşağı akıştaki her şeyin istediği cevap |
| `v, ω` | **İki bağımsız sensör** bunları ölçüyor: encoder ikisini de, jiroskop `ω`'yı |
| `b_gyro` | Jiroskopun yavaş kayan sıfır noktası |

### `v` ve `ω` neden *durum*, *girdi* değil?

Yaygın kurulum odometriyi **kontrol girdisi** (`u`) olarak alır. O zaman
jiroskopun bilgisi **atılır** — çünkü füze edilecek bir durum yoktur. Phase 3
tüm kalan hatanın dönüşte olduğunu ölçtü; jiroskopu atmak tam olarak
ihtiyacımız olan bilgiyi atmak demek.

Durum yapınca filtre iki görüşü **ağırlıklandırarak birleştirir**.

### `b_gyro` neden var ve neden **gözlemlenebilir**?

Jiroskop bias'ı gürültü değildir: `t` saniye integre edilince `b·t` kadar
**sınırsız** heading hatası üretir. Phase 2'de bunu Ornstein-Uhlenbeck süreci
olarak modelledik, çünkü ortalamayla yok edilemez.

Burada gözlemlenebilir, çünkü **encoder `ω`'yı bağımsız ölçüyor**:

```
encoder:  ω          = 0.50
jiroskop: ω + b      = 0.53
                  →  b = 0.03
```

Yalnızca jiroskop olsaydı, toplamı okumayı açıklayan her `(ω, b)` çifti eşit
derecede geçerli olurdu — gözlemlenemez.

### Kasten YOK olanlar

**`v_y` (yanal hız).** Diferansiyel sürüş **holonomik olmayan**: yana
gidemez, `v_y ≡ 0`. Eklersen hiçbir ölçümün kısıtlamadığı bir durum yaratırsın;
filtre o yönü süreç gürültüsüyle doldurur, kovaryans sınırsız büyür ve pozisyon
yavaşça yana yayılır. **Gözlemlenemeyen durum bedavaya gelmez.**

**İvmeölçer bias'ı.** İvmeölçer ölçüm modelinde hiç kullanılmıyor. Düzlemsel bir
robotta yürüme hızında faydalı sinyal, milideredelik şasi eğiminden sızan
yerçekiminin altında kalır; iki kez integre edip pozisyon çıkarmak tekerlek
encoder'ından kesinlikle daha kötüdür. Yayınlanıyor, loglanıyor, **inanılmıyor**.

---

## 2. Prediction — Phase 3'ün aynı üstel haritası

```
a = ω·dt,  half = a/2,  k = sinc(half)
x'  = x + v·dt·k·cos(θ + half)
y'  = y + v·dt·k·sin(θ + half)
θ'  = θ + a
v'  = v        ← ivme gürültüsüyle sürülür
ω'  = ω        ← açısal ivme gürültüsüyle sürülür
b'  = b        ← rastgele yürüyüş
```

"Sabit hız modeli", robotun hiç ivmelenmediği iddiası **değildir**. Filtrenin
komut edilen ivmeyi bilmediğini, dolayısıyla değişimi gürültü olarak
modellediğini söyler. O gürültünün boyutunu `accel_sigma` ve
`angular_accel_sigma` beyan eder.

### Jacobian — `∂x'/∂ω` iki yoldan gelir

`ω` hem `sinc` çarpanında hem orta-nokta yönünde görünüyor, her biri `dt/2`
katkı veriyor:

```
∂x'/∂ω = v·dt²/2 · [ sinc'(half)·cos(θ+half) − sinc(half)·sin(θ+half) ]
```

Terimlerden birini düşürmek durumda **görünmez**, kovaryansta yanlıştır. Merkezi
sonlu fark testi ikisini de yakalıyor.

---

## 3. BULUNAN BUG #1 — Süreç gürültüsünün `dt` ölçeklemesi

Bu, projenin şimdiye kadarki en ince ve en pahalı hatasıydı.

Yazdığım:
```cpp
const double dv = accel_sigma * dt;
Q(V,V) = dv * dv;              // = σ²·dt²   YANLIŞ
```

Doğrusu:
```cpp
Q(V,V) = accel_sigma * accel_sigma * dt;   // = σ²·dt
```

### Neden?

Beyaz ivme gürültüsüyle sürülen bir hız **rastgele yürüyüştür**. Varyansı
`q·T` ile büyür ve **integrasyon adımından bağımsızdır**: 20 ms'lik bir adımı
dört 5 ms adıma bölmek, 20 ms sonraki belirsizliği değiştirmemeli.

`(σ·dt)²` yazınca birikim `σ²·T·dt` olur — adım küçüldükçe **küçülür**. Aynı
filtreyi 50 Hz yerine 200 Hz'de çalıştırmak onu dört kat daha özgüvenli yapar,
hiçbir fiziksel sebep olmadan.

### Ölçülen sonuç

| | 20 ms'de biriken σ_v |
|---|---|
| Yanlış `(σ·dt)²`, dt=5 ms | **0.008 m/s** |
| Doğru `σ²·dt` | **0.113 m/s** |

Platformun ivme limiti 1.0 m/s² → 20 ms'de `Δv = 0.020 m/s`.

- Yanlış model: `0.020/0.008 = 2.5σ` → **NIS ≈ 6**, gate eşiği 9.21'e çok yakın
- Doğru model: `0.020/0.113 = 0.18σ` → NIS ≈ 0.03, rahat kabul

### Belirti

Filtre robotun **gerçek yavaşlamasını aykırı değer sanıp reddediyordu.**
Yerinde dönüşte:

```
faz       odom v    ekf v    odom w    ekf w
donus     0.0002   0.0803    0.6655   0.6680     ← hayalet 8 cm/s ileri hız
```

Değerlendirme rotasında EKF **her köşede yarım metrelik halka** çiziyordu:
robot yerinde dönerken filtre hâlâ ileri gittiğine inanıyordu.

Pozisyon RMSE: odometri 0.049 m, EKF **0.298 m** — filtre odometriden **6 kat
kötüydü**.

Düzeltmeden sonra: EKF **0.0155 m**, odometriden **%68 daha iyi**.

> **Ders:** Süreç gürültüsü "ayarlanacak bir sayı" değil, **fiziksel bir
> model**. Yanlış `dt` kuvveti filtreyi hız-bağımlı yapar ve hatayı
> katsayıyı büyüterek gizleyebilirsin — ama o zaman 200 Hz'de doğru,
> 50 Hz'de yanlış olur.

---

## 4. BULUNAN BUG #2 — Çok hızlı sensör füzyonu ve eşit zaman damgaları

İlk çalıştırmada:

```
odom 3 (0 rejected)   imu 1917 (0 rejected)   out-of-order 485
```

**10 saniyede 3 odometri güncellemesi.** Filtre pratikte yalnızca jiroskopla
çalışıyordu — bu da jiroskop bias'ını **gözlemlenemez** yapar, yani altı-durumlu
tasarımın var olma sebebi yok oluyordu. Hiçbir hata mesajı yok.

### Sebep 1 — çok hızlı akışlar

IMU 200 Hz, odometri 50 Hz. Hangi callback tetiklenirse filtre saatini kendi
damgasına taşıyor. Bu yüzden odometri hep "geçmişte" kalıyor.

**Çözüm:** ölçüm tamponu. Ölçümler kısa süre bekletilir, sonra **zaman damgası
sırasıyla** işlenir. Maliyet: sabit gecikme (burada 60 ms, ölçüldü: medyan
55 ms). Kazanç: her sensör **ölçtüğü andaki** duruma karşı füze edilir.

### Sebep 2 — eşit damgalar

Tampon eklendikten sonra bile `odom 3` devam etti. Gazebo 1 ms fizik ızgarasında
yayın yapıyor, yani 200 Hz IMU ile 50 Hz encoder **her dördüncü örnekte tam
aynı zaman damgasını paylaşıyor**. Kodum:

```cpp
if (stamp <= filter_time_) { ++out_of_order_; return false; }   // YANLIŞ
```

IMU önce uygulanıp `filter_time_ = T` yapınca, aynı `T` anındaki odometri
"eski" sayılıp atılıyordu.

**Çözüm:** `dt == 0` geçerli bir durumdur — tahmin gerekmez, sadece güncelleme:

```cpp
if (dt == 0.0) { return true; }      // aynı anda ölçülen iki sensör
if (dt <  0.0) { ++out_of_order_; return false; }
```

Sonrası: `odom 467/470 fused, imu 1833/1845 fused, out-of-order 0`.

---

## 5. Joseph formu — neden ders kitabı güncellemesi yetmez

```
P ← (I − KH)·P                          ders kitabı
P ← (I − KH)·P·(I − KH)ᵀ + K·R·Kᵀ      Joseph
```

İkisi **tam aritmetikte özdeş**. Farkı yuvarlama yapar.

Ders kitabı formu iki pozitif niceliğin **farkı**dır; yuvarlama `P`'yi
asimetrik ve sonunda **belirsiz** yapabilir. O noktada filtre negatif varyans
raporlar ve bir sonraki Cholesky çarpanlaması başarısız olur.

Joseph formu iki **pozitif yarı-belirli** terimin toplamıdır — hiçbir yuvarlama
onu belirsiz yapamaz.

### Testi keskinleştirmek

İlk testim (2000 adım, simetri + özdeğer kontrolü) mutasyonu **yakalayamadı**.
Çünkü `symmetrise()` farkı maskeliyor ve normal koşullarda çift hassasiyet
yetiyor.

Ayırt eden test **fiziksel bir sınır**:

> Ölçüm, önsel bilgiden çok daha hassassa, **sonsal varyans ölçüm varyansını
> aşamaz.**

Önsel `1e6`, `R = 1e-14`. Ders kitabı formu `~2e-10` üretiyor — `R`'nin **10⁴
katı** ve işareti rastgele. Joseph `1e-14` veriyor. Tolerans yerine sınır
koyunca mutasyon anında yakalandı.

---

## 6. NIS kapısı — filtrenin bağışıklık sistemi

```
NIS = yᵀ·S⁻¹·y  ~  χ²(dim z)
```

Bir EKF'in yanlış ölçüme karşı **başka savunması yoktur**: yanlış eşleşen bir
scan match'i seve seve füze eder, ve kestirim kaydıktan sonra bir sonraki eşleşme
daha da yanlış olur. Kapı, ıraksamayı **görünür bir reddetmeye** çevirir.

χ² %99 kuantilleri: 6.63 (1 sdd), 9.21 (2 sdd), 11.34 (3 sdd).

### Kapı bir tuzağa da dönüşebilir

Filtre **gerçekten kaybolmuşsa**, her doğru ölçüm aykırı görünür ve sonsuza
kadar reddedilir. Bu yüzden `max_consecutive_rejections` sonrası kapı bir kez
**zorla açılır**.

Bunu test yazarken canlı yaşadım: bias önsel'ini 0.1 rad/s (saniyede 5 derece,
herhangi bir MEMS için saçma) verip `ω`'yı 0'dan başlatınca, ilk çevrimde
encoder okuması 5σ uzaktaydı ve **reddedildi**; jiroskop okuması ise dev bias
önseliyle açıklanabildiği için kabul edildi. Bias tüm 0.5 rad/s'yi soğurup
**0.475'te** yerleşti ve sonrasında her encoder ölçümü aykırı kaldı. Filtre
yanlış ama **kusursuz derecede tutarlı** bir duruma kilitlendi.

> **Kapı, kabaca doğru olan bir filtreyi korur; yanlış başlayan bir filtreyi
> hapseder.** Bu yüzden gerçek node **ilk odometri mesajından** başlatılır.

---

## 7. `map → odom` düzeltmesi

Filtrenin cevabı `base_link`'in `map`'teki pozu. Ama TF ağacında
`odom → base_link` zaten var ve bir çerçevenin **tek ebeveyni** olabilir. O
yüzden EKF şunu yayınlar:

```
T_map_odom = T_map_base · T_odom_base⁻¹
```

Böylece `odom → base_link` kontrolcü için **pürüzsüz**, `map → base_link`
bileşimi planlayıcı için **global tutarlı** kalır.

---

## 8. PHASE 5 — Ölçülen sonuçlar

10.03 m, 2 tur dikdörtgen rota:

| Metrik | Odometri | EKF | Değişim |
|---|---|---|---|
| Pozisyon RMSE | 0.0486 m | **0.0155 m** | **−%68.0** |
| Pozisyon ortalama | 0.0393 m | 0.0139 m | −%65 |
| Pozisyon maks | 0.0949 m | 0.0358 m | −%62 |
| Son pozisyon hatası | 0.0946 m (%0.94) | **0.0134 m (%0.13)** | −%86 |
| Heading RMSE | 3.631° | **1.432°** | **−%60.6** |
| Heading maks | 6.811° | 3.225° | −%53 |
| Son heading hatası | +6.626° | **+0.977°** | −%85 |
| Raporlanan son 1σ (poz) | 0.2436 m | 0.1175 m | — |
| Raporlanan son 1σ (yön) | 13.26° | 3.23° | — |
| 2σ içinde kalma (poz) | %100 | %100 | — |

**Kestirim gecikmesi:** medyan 55 ms, p95 60 ms, maks 70 ms — ölçüm tamponunun
bedeli, gizlenmiyor, ölçülüyor.

### Kovaryans grafiği ne anlatıyor?

- Odometrinin hatası **ve** raporlanan belirsizliği sürekli büyüyor
- EKF'in hatası **düz** kalıyor, belirsizliği çok yavaş büyüyor
- Heading'de odometri 6.6°'ye basamaklanıyor; EKF 3°'nin altında kalıyor

Jiroskop, heading belirsizliğinin **büyüme hızını** sınırlıyor. Ama mutlak bir
ölçüm olmadığı için pozisyon belirsizliği hâlâ sınırsız — sadece çok daha
yavaş. **Mutlak sınır, Phase 6'da harita geldiğinde LiDAR scan matching ile
gelecek.**

---

## 9. Tutarlılık — filtrenin kendi belirsizliği bir şey ifade ediyor mu?

Kritik saha teşhisi, **ground truth gerektirmez**:

```
ortalama NIS  ≈ dim(z)     tutarlı
ortalama NIS >> dim(z)     aşırı özgüvenli: R veya Q çok küçük, ıraksayacak
ortalama NIS << dim(z)     muhafazakâr: bilgi atıyor
```

`EkfConsistency.AverageNisMatchesTheMeasurementDimension` testi bunu 20.000
güncellemede ölçüyor ve 2.0 ± 0.15 bekliyor.

---

## 10. Mutasyon testi — 10/10

| # | Mutasyon | İlk | Son |
|---|---|---|---|
| M1 | Joseph yerine `(I−KH)P` | ❌ | ✅ |
| M2 | Jiroskop modelinde bias terimi yok | ✅ | ✅ |
| M3 | Innovation işareti ters | ✅ | ✅ |
| M4 | Heading residual sarılmıyor | ✅ | ✅ |
| M5 | `F(X,ω)` sinc türevi yok | ✅ | ✅ |
| M6 | `F(θ,ω) = 0` | ✅ | ✅ |
| M7 | Bias gürültüsü `dt` yerine `dt²` | ✅ | ✅ |
| M8 | Hız gürültüsü `dt` ile ölçeklenmiyor | ✅ | ✅ |
| M9 | `predict` θ'yı sarmıyor | ✅ | ✅ |
| M10 | Kapı her şeyi kabul ediyor | ✅ | ✅ |

---

## Mini görev

> `ros2_ws/src/arb_localization/config/localization.yaml` içinde
> `process.accel_sigma`'yı `0.8` → `0.05` yap (platform limiti 1.0 m/s²).
> Derle, stack'i başlat, `scripts/evaluate_localization.py --tag lowQ` çalıştır.
>
> 1. Pozisyon RMSE ne oluyor?
> 2. `ros2 topic echo /ekf/diagnostics` çıktısında `update_accepted` ve
>    `normalized_innovation_squared` ne gösteriyor?
> 3. EKF node'unun 10 saniyelik raporunda `odom ... (N gated)` sayısı ne?
> 4. **Bunu `gating.nis_2dof`'u büyüterek "düzeltmek" doğru olur muydu?
>    Neden olmaz?**
