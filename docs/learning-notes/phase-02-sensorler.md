# Phase 2 — Sensör Arayüzleri, Gürültü Modeli ve Arıza Enjeksiyonu

Bu fazda ham simülatör verisini alıp **gerçek bir sensörün üretebileceği**
veriye dönüştüren üç C++ node yazdık. Ama asıl öğretici kısım yine bulduğumuz
hatalar: **mutasyon testi** ve **canlı denetim** olmasa dördü de sessizce
geçecekti.

---

## 1. Mimari karar: saf mantık ile ROS bağlantısını ayır

```
arb_sensors/
├── include/arb_sensors/
│   ├── noise_model.hpp        ← RandomSource, GaussMarkovBias, ChannelNoise
│   ├── imu_processor.hpp      ← saf mantık, rclcpp YOK
│   ├── encoder_processor.hpp
│   └── lidar_processor.hpp
└── src/
    ├── *_processor.cpp        ← libarb_sensors_core.so
    └── *_preprocessor_node.cpp ← ince ROS sarmalayıcı
```

Neden: işlem sınıfları `rclcpp`'ye bağlı değil, bu yüzden gtest ile **node
başlatmadan, graf kurmadan, 0.4 saniyede** test edilebiliyorlar. Node'lar
yalnızca parametre okuyup pub/sub bağlıyor.

Bir ROS node'unu test etmek için `rclcpp::init` + executor + sahte publisher
kurmak zorunda kalsaydın, 83 testin hiçbirini yazmazdın. Test edilebilirlik bir
mimari özelliktir, sonradan eklenemez.

---

## 2. Gürültü modeli — üç ayrı bozulma, üç ayrı sonuç

```
ölçüm = (1 + ölçek_hatası) · gerçek + bias(t) + beyaz_gürültü
```

Neden üçü **ayrı** modelleniyor:

| Bileşen | Zamanla davranışı | Estimator için sonucu |
|---|---|---|
| Beyaz gürültü | Ortalaması sıfır, bağımsız | **Ortalama alınca yok olur.** EKF'in `R` matrisi halleder |
| Bias | Yavaş değişir, neredeyse sabit | **Ortalama alınca yok OLMAZ.** İntegre edilince `b·t` kadar sınırsız hata |
| Ölçek hatası | Sinyalle orantılı | Sadece robot hareket ederken görünür — "gelip gidiyor" sanılır |

Bu ayrım tüm estimator tasarımının temeli. `test_noise_model.cpp` içinde
`WhiteNoiseIsZeroMeanSoItAveragesAwayButBiasDoesNot` testi tam olarak bunu
kilitliyor.

### Bias neden Ornstein-Uhlenbeck?

Gerçek MEMS sensörlerin sıfır noktası gezinir: saniyeler ölçeğinde sabit,
dakikalar ölçeğinde kayar. Standart model birinci mertebe Gauss-Markov:

```
db/dt = -(1/tau)·b + sigma_w·w(t)
```

Ayrık zamanda **tam** çözümü:

```
b_{k+1} = b_k·e^(-dt/tau) + sigma_ss·sqrt(1 - e^(-2dt/tau))·n
```

`tau <= 0` verilince saf rastgele yürüyüşe dönüşür (ortalamaya dönüş yok) — en
kötü durum, ve "encoder drift" arızasının kullandığı model.

**Neden tam çözüm, neden naif Euler değil?** Naif adım
`sigma_ss·sqrt(2·dt/tau)` kullanır. Küçük `dt/tau`'da ikisi aynıdır, bu yüzden
sadece küçük adımlarla test edersen **farkı göremezsin**:

| dt/tau | naif Euler hatası |
|---|---|
| 0.001 | %0.1 |
| 0.01 | %0.5 |
| 0.1 | %5 |
| 1.0 | **%52** |
| 2.0 | **%102** |

İlk testim `dt/tau ∈ [0.001, 0.1]` kullanıyordu ve toleransı %30'du. Naif Euler
mutasyonunu **yakalayamadı**. Testi tek adımın **koşullu dağılımını** ölçecek
şekilde yeniden yazdım:

```
E[b_{k+1} | b_k]   = b_k·e^(-dt/tau)
Std[b_{k+1} | b_k] = sigma_ss·sqrt(1 - e^(-2dt/tau))
```

Bu hem keskin hem de karışma süresinden bağımsız.

---

## 3. Tekrarlanabilirlik: tohum yönetimi

```cpp
RandomSource(std::uint32_t base_seed, const std::string & stream_name)
```

Her sensör **kendi akışını** `stream_name`'i tohuma karıştırarak türetiyor
(FNV-1a). Sebebi:

- `std::random_device` veya saat kullanırsan **benchmark tekrarlanamaz**,
  regresyon bisect edilemez, "RMSE kötüleşti" ile "bu koşu şanssız örnekler
  çekti" ayırt edilemez.
- Tüm sensörler aynı akışı paylaşırsa, **LiDAR config'ini değiştirmek IMU'nun
  gürültü dizisini kaydırır**. "Sadece LiDAR'ı değiştirdim" cümlesi yalan olur.

---

## 4. BULUNAN BUG #1 — Gürültü iki kez uygulanıyordu

Gazebo, URDF'teki `<noise>` bloğu sayesinde datasheet gürültüsünü **zaten**
ekliyor. Bizim katman da aynı sigma'yı ekleyince:

```
toplam = sqrt(sigma² + sigma²) = 1.41·sigma
```

Akış iddia ettiğimiz parçadan **%41 daha gürültülü**, ve yayınladığımız her
kovaryans aynı çarpanla yanlış. Phase 4'teki EKF ilk satırından itibaren yanlış
ayarlanmış olurdu.

**Canlı ölçüm:** gyro std **0.007112** rad/s, config ise 0.004950. Tam √2 kat.

### Çözüm: bağımsız varyanslar toplanır

Girdi zaten `sigma` taşıyorsa, toplam `noise_scale·sigma` hedefine ulaşmak için
eklenmesi gereken:

```
eklenen_sigma = sqrt((noise_scale·sigma)² - sigma²)
```

`noise_scale = 1`'de bu **tam sıfır** — nominal çalışmada hiçbir şey eklenmez ve
simülatörün kendi modeli korunur.

Kodda: `ChannelNoise::added_sigma()` ve `LidarProcessor::added_range_sigma()`.

**Düzeltme sonrası:** gyro std **0.005019** (config 0.004950 ✓),
`noise_scale=10` → **9.7×** (öncesi 6.8×).

---

## 5. BULUNAN BUG #2 — Encoder çıkışı sayım ızgarasında değildi

Quadrature encoder **tam sayı sayım** raporlar, başka bir şey değil.
1024 × 4 = 4096 sayım/tur → çözünürlük 2π/4096 = **1.534 mrad**
(60 mm tekerlekte **0.092 mm yay**).

İlk versiyonum gerçeği kuantalıyor, sonra **kuantalanmamış** kayma ve gürültü
ekliyordu. Sonuç: hiçbir gerçek encoder'ın üretemeyeceği pozisyonlar.

**Canlı ölçüm: 598/598 örnek ızgara dışında.**

### Çözüm ve içindeki tuzak

Artışı değil, **biriktiriciyi** yuvarla:

```cpp
left_accumulator_ += d_left;              // tam hassasiyet
const double left_out = quantise(left_accumulator_);   // yayında yuvarla
```

Neden artışı yuvarlamak yanlış: 0.031 rad/s'lik bir drift, 20 ms adımda
0.00062 rad ilerler — **yarım sayımın altında**. Her artışı yuvarlarsan hepsi
sıfıra yuvarlanır ve **arıza hiç görünmez**. Test:
`SubCountDriftStillAccumulatesDespiteQuantisation`.

### Bonus: encoder hızı ölçülmez, türetilir

Bu çok sayıda kişinin kaçırdığı nokta:

```
hız = Δ(sayım) / Δt        →  kuantalama hatası = çözünürlük / dt
```

50 Hz'de: 0.001534 / 0.02 = **0.0767 rad/s**. Tekerlek 0.7 rad/s dönüyorsa bu
**%11 gürültü**. Yani encoder hızı **düşük hızda gürültülü, yüksek hızda
temiz** — çoğu kişinin sezgisinin tam tersi. Phase 3'te bu yüzden hızlara
güvenmeyip **pozisyonları integre edeceğiz**.

---

## 6. BULUNAN BUG #3 — Despeckle gerçek veriyi siliyordu

Filtre "iki komşusu da uzakta olan ışını sil" diyordu, eşik sabit **0.25 m**.

**Canlı denetim:** her taramada 12 ışın siliniyor, ve silinenlerin **%99.9'u
gerçek bir duvarın 5 cm yakınında**. Filtre gürültü değil, **veri** siliyordu.

### Neden — geometri

Düz bir yüzeye **yalayarak** bakıldığında komşu ışınlar arasındaki menzil farkı:

```
Δr = r · tan(φ) · Δθ        (φ = yüzey normalinden sapma)
```

| r | φ=45° | φ=75° | φ=85° |
|---|---|---|---|
| 1 m | 0.017 | 0.065 | 0.199 |
| 5 m | 0.087 | **0.326** | **0.997** |
| 10 m | 0.175 | **0.651** | **1.995** |

10 m'de 75° eğimli bir duvar komşu ışınlar arasında meşru olarak **0.65 m**
atlar. Sabit 0.25 m eşiği hepsini "aykırı değer" sayar — üstelik taramanın
**en bilgi taşıdığı** yerde, koridorun uzağında.

### Çözüm: eşiği geometriden türet

```cpp
threshold = max(floor, r · tan(max_obliquity) · angle_increment)
```

`max_obliquity = 85°` → `tan = 11.43`. Düzeltme sonrası **sıfır ışın siliniyor**
ve 43.200 ışın korunuyor.

---

## 7. BULUNAN BUG #4 — Arıza knob'u kalibre değildi

`dropout_ratio = 0.6` komutu verildiğinde gerçekte **%75.5** ışın kayboluyordu.

Sebep sıralama: önce dropout, sonra despeckle. %60 ışın rastgele silinince
hayatta kalanların çoğunun komşusu da silinmiş oluyor → despeckle onları da
siliyor.

### Neden önemli

Phase 10 benchmark tablosunda **"%60 dropout"** yazan bir satır aslında başka
bir şeyi ölçüyor olurdu. **Arıza enjektörü bir ölçüm aletidir; ayarı aşağı
akışta sessizce büyütülen bir alet kalibre değildir.**

Çözüm: despeckle → gürültü → **dropout en sona**. Düzeltme sonrası: %60 komut →
**%39.2 hayatta** (beklenen %40 ✓).

---

## 8. Kritik tasarım: arıza kovaryansa da yansımalı

Phase 10'un kilit noktası. IMU bozulduğunda EKF'e **söylenmeli**, yalnızca daha
kötü sayı beslenmemeli:

```cpp
// effective_sigma() = white_sigma_ * noise_scale_   (TOPLAM)
set_diagonal_covariance(out.angular_velocity_covariance, gyro_sigma());
```

Sigma büyüyüp raporlanan kovaryans büyümezse, EKF bozulmuş bir sensöre güvenmeye
devam eder ve **ıraksar**. Test: `NoiseScaleWidensTheReportedCovarianceToo` —
mutasyon testinde bu mutasyon yakalandı.

Ayrıca: Gazebo kovaryansı **sıfır** yayınlıyor. ROS 2'de sıfır kovaryans
"bilinmiyor" demektir, "mükemmel" değil. Doldurmasak her tüketici kendi `R`
matrisini uydururdu.

Ve `orientation_covariance[0] = -1`: bu bir AHRS değil, sadece jiroskop +
ivmeölçer. Bayrağı görmezden gelip birim quaternion kullanan bir tüketici,
**hep +x'e baktığını sanan** bir robot elde eder.

---

## 9. LaserScan'in tek kuralı

Bir menzil her zaman sayı **değildir**:

| Değer | Anlamı |
|---|---|
| `r > range_max` (genelde `+inf`) | Işın hiçbir şeye çarpmadı |
| `r < range_min` (genelde `-inf`) | Bir şey ölçülemeyecek kadar yakın |
| `NaN` | Okuma geçersiz |

Bunlar **birbirinin yerine geçmez** ve hiçbiri mesafe değildir.

- `if (r < 0.5) engel_var()` → her `-inf` ışında engel raporlar
- Taramanın ortalamasını almak → tek `NaN` tüm sonucu `NaN` yapar

Preprocessing hepsini **tek sentinel**'e (`+inf`) normalize ediyor, böylece
aşağı akışta ele alınacak tek bir durum kalıyor.

---

## 10. Mutasyon testi — testlerin testi

52 test ilk seferde geçti. Bu beni şüphelendirdi, çünkü **geçen bir test
hiçbir şey kanıtlamaz**. Kasten 5 hata enjekte ettim:

| # | Mutasyon | İlk turda | Son turda |
|---|---|---|---|
| M1 | Despeckle wrap-around'u kaldır | ❌ kaçtı | ✅ yakalandı |
| M2 | Encoder kuantalamasını kaldır | ✅ | ✅ |
| M3 | Kovaryans `noise_scale`'i yansıtmasın | ✅ | ✅ |
| M4 | OU yerine naif Euler | ❌ kaçtı | ✅ yakalandı |
| M5 | Bias `dt` ile çarpılmasın (sol tekerlek) | ❌ kaçtı | ✅ yakalandı |

**M5 neden kaçtı?** Mutasyonu **sol** tekerleğe uyguladım, testim yalnızca
**sağ** tekerleği kontrol ediyordu. Sol ve sağ ayrı kod yolları. Testi iki
tekerlek için parametrize ettim (`INSTANTIATE_TEST_SUITE_P`).

**M1 neden kaçtı?** Mutasyonum `before[i>0 ? i-1 : 0]` idi; `i=0` için ışını
**kendisiyle** karşılaştırıyor, fark 0 → "yakın" → korunuyor. Testim ışın 0'ın
*korunmasını* bekliyordu, o yüzden geçti. Işın 0'ın **kendisinin speckle
olduğu** bir vaka ekledim.

Ders: mutasyon testi sadece kodu değil, **test tasarımını** da denetler.

---

## 11. Bulduğum bir ROS tuzağı

```python
self._clients = {...}   # rclpy.node.Node'un İÇ alanını eziyor
```

`rclpy.Node` kendi servis istemci listesini `self._clients` içinde tutuyor.
Üzerine dict yazınca executor string'ler üzerinde döndü ve
`destroy_node()` bir dict'i integer ile indeksledi. İki hata da atamadan
**çok uzakta** patladı.

Ders: bir framework sınıfından türerken `_` ile başlayan alan adlarını kullanma.

---

## Mini görev

> `configs/sensors.yaml` içinde `wheel_encoder.counts_per_revolution`'ı
> **1024 → 64** yap. Derle, stack'i başlat, `scripts/verify_sensors.py`
> çalıştır.
>
> 1. Yayınlanan hız sigma'sı (`effort[0]`) ne oluyor? Neden?
> 2. `positions lie on the count grid` testi hâlâ geçiyor mu?
> 3. `tests/integration/test_config_consistency.py` ne diyor? **Neden?**
>
> İpucu: 64 × 4 = 256 sayım/tur → çözünürlük 2π/256. Bunu 60 mm tekerlek
> yarıçapıyla çarp.
