# Phase 0 — Ortam, Mimari ve Açı Matematiği

## 1. Neden ROS 2 Jazzy + Gazebo Harmonic?

Bu bir "hangisini seversem" kararı değil, **uyumluluk matrisi** kararı.

ROS dağıtımları belirli Ubuntu sürümlerine kilitlidir; çünkü ROS, sistem
kütüphanelerine (Python sürümü, Boost, TinyXML2, OpenCV...) apt üzerinden
bağlanır. Yanlış eşleştirme = kaynaktan derleme = saatlerce bağımlılık savaşı.

| Ubuntu | ROS 2 | Gazebo | Durum |
|---|---|---|---|
| 22.04 Jammy | Humble | Fortress | LTS, 2027'ye kadar |
| **24.04 Noble** | **Jazzy** | **Harmonic** | **LTS, 2029'a kadar ← bizim durumumuz** |
| 24.04 Noble | Humble | — | Desteklenmiyor, kaynak derleme gerekir |

Senin makinen Noble olduğu için Jazzy tek mantıklı seçim.

**Gazebo isim karmaşası** (mülakatlarda sorulur): "Gazebo Classic" (gazebo11)
Ocak 2025'te EOL oldu. Yerine gelen yeni mimariye önce "Ignition", sonra tekrar
"Gazebo" dendi — sürümler harf sırasıyla ilerliyor: Fortress → Garden →
Harmonic → Ionic. Komut adı `gz`, eski `gazebo` değil. Yeni Gazebo **ECS**
(Entity-Component-System) mimarisi kullanır: bir robot, "entity"dir; sensörler,
gövde, ataletler ona takılı "component"lerdir; fizik/render/sensör sistemleri
her adımda bu component'leri işler. Bu, klasik Gazebo'nun monolitik plugin
modelinden daha test edilebilir bir yapıdır.

## 2. Repo mimarisi — neden 11 paket?

Senin önerdiğin iskelete iki ekleme yaptım. Gerekçeleri:

### `arb_core` (yeni)

Açı normalizasyonu, SE(2) poz bileşimi ve koşan istatistikler **dört ayrı
yerde** lazım: odometri, EKF, planlayıcı, kontrolcü. Ortak paket olmazsa her
biri kendi kopyasını üretir, kopyalar zamanla birbirinden ayrışır ve birinde
düzelttiğin bug diğer üçünde yaşamaya devam eder.

Header-only (`INTERFACE` CMake target) yaptım: link maliyeti yok, ABI yüzeyi
yok, sadece include path miras alınır.

### `arb_msgs` (ayrı paket)

ROS 2'de interface paketleri, onları yayınlayan node'lardan **ayrı** olmalı.
Aksi halde mesajı *tüketen* her paket, *üreten* node'un tüm bağımlılık ağacını
içeri çeker. `colcon build --packages-up-to arb_msgs` saniyeler sürer, dakikalar
değil.

### Config neden iki yerde?

- `configs/` (repo kökü) → **deney seviyesi**: robotun fiziksel sabitleri, altı
  arıza senaryosu. Bunları ROS paketi olmayan scriptler ve analiz araçları okur.
- `<paket>/config/` → **node seviyesi**: ROS parametreleri. `ament_index` ve
  `launch_ros` bunları burada arar.

Ayrım şunu sağlar: "hangi deneyi çalıştırıyorum" sorusunu değiştirmek için
"bu node nasıl tune edilmiş" dosyasına dokunmak zorunda kalmazsın.

### Test neden iki yerde?

- `<paket>/test/` → **unit test**. `colcon test` otomatik bulur.
- `tests/integration/` → **sistem testi**. Tüm node grafiğini inceleyen
  testler hiçbir tek pakete ait değildir.

## 3. Ground truth izolasyonu — projenin en önemli kuralı

Gazebo robotun gerçek pozunu biliyor. Bu bilgiyi `/arb/ground_truth/pose`
üzerinden yayınlıyoruz ama **yalnızca değerlendirme için**.

Neden bu kadar ciddiye alıyoruz: ground truth'un estimator'a sızması, robotik
demolarının anlamsızlaşmasının **bir numaralı** yolu. Trajectory grafiği
kusursuz çıkar, RMSE 2 mm olur, ve sistem gerçek donanımda hiç çalışmaz.
Üstelik hata **sessizdir** — aramazsan fark etmezsin.

Bu yüzden kural niyet değil, **mekanik kontrol**:
`tests/integration/test_ground_truth_isolation.py` hem kaynak kodu tarar hem de
canlı node grafiğindeki abonelikleri inceler.

> **Önemli detay:** Bu testi yazdıktan sonra *kasıtlı bir ihlal* ekleyip testin
> gerçekten patladığını doğruladım. Bir guard testinin yazılması yetmez;
> **yakalamadığını kanıtlamak** gerekir. Yoksa boş yere geçen bir test, olmayan
> bir güvenlik hissi verir.

## 4. Açı matematiği — neden `theta` sıradan bir sayı değil?

Bu, Phase 0'ın gerçek teknik içeriği.

### Problem

Robotun yönü `theta`, reel sayı doğrusunda (**R**) değil, **çember üzerinde**
(**S¹**) yaşar. 2π farklı iki sayı **aynı fiziksel yönü** tarif eder:

```
theta = 0.1 rad   ve   theta = 0.1 + 2π rad   →   aynı yön
```

Ama bilgisayar için bunlar farklı sayılar. Bu uyumsuzluk iki yerde patlar:

**Kontrolcüde:** Robot +179°'de, hedef −179°'de.
```
naif hata = -179 - 179 = -358°
gerçek hata = +2°
```
Kontrolcü 2° yerine 358° dönmeye çalışır — robot ters yöne tam tur atar.

**EKF'te:** İnnovation `y = z − h(x̂)` aynı hatayı yapar. Sahte dev innovation,
Kalman kazancını olmayan bir hataya tepki vermeye zorlar ve **filtre ıraksar**.

### Çözüm

Açı içeren her çıkarma işlemi S¹'e geri projekte edilmeli:

```
normalizeAngle(a) = a − 2π · round(a / 2π)      → [−π, π)
angleDiff(to, from) = normalizeAngle(to − from)
```

Kodda: [`arb_core/include/arb_core/angles.hpp`](../../ros2_ws/src/arb_core/include/arb_core/angles.hpp)

### Neden `std::remainder`, neden `while` döngüsü değil?

Çoğu tutorial şunu yazar:

```cpp
while (a >  M_PI) a -= 2*M_PI;   // KÖTÜ
while (a < -M_PI) a += 2*M_PI;
```

Üç problemi var:
1. **O(n)**: `a = 1000·2π` girerse ~1000 iterasyon. Gerçek zamanlı döngüde
   belirsiz süre.
2. **Sonsuz döngü**: `a = inf` veya `a = NaN` girerse **asla bitmez**. Bozuk bir
   IMU mesajı tüm node'u dondurur.
3. **Hassasiyet kaybı**: Tekrarlanan çıkarma, kayan nokta hatasını biriktirir.

`std::remainder(a, 2π)` ise O(1), IEEE-754 tarafından tam tanımlı ve büyük
girdilerde bile hassas. Biz ek olarak `inf`/`NaN` için açıkça `NaN` döndürüyoruz
— sessizce kilitlenmek yerine, gürültülü şekilde başarısız olmak.

### `Pose2D` neden bir struct, neden 3-vektör değil?

Düzlemsel poz, SE(2) Lie grubunun bir elemanıdır. Bileşim **toplama değil,
matris çarpımıdır**:

```
     | cos θ   −sin θ   x |
 T = | sin θ    cos θ   y |
     |   0        0     1 |
```

`(x, y, θ)` olarak saklamak pratiktir ama **tipi ayırmak**, bileşen bazında
toplama/ortalama alma klasik hatasını engeller. `arb_core::Pose2D`'de
`compose()`, `inverse()` ve `relativeTo()` doğru grup işlemleridir ve `theta`
her zaman normalize edilir.

`relativeTo()` özellikle önemli: "hedef, robot çerçevesinde nerede?" sorusunun
doğru cevabı `robot.inverse() ∘ hedef`'tir. Phase 9'da kontrolcü tam olarak
bunu kullanacak.

## 5. QoS — sensör verisi neden BEST_EFFORT?

ROS 2, DDS üzerine kurulu ve QoS (Quality of Service) profilleri gerçek bir
mühendislik kararı.

- **Sensör akışları → BEST_EFFORT**: Düşen bir LiDAR taraması, *geç gelen* bir
  taramadan iyidir. Yeniden iletim, gerçek zamanlı filtreye **bayat** veri
  teslim eder — bu, veri kaybından daha zararlıdır.
- **`/cmd_vel` → RELIABLE, depth 1**: Kayıp bir komut, robotun önceki hızını
  sürdürmesi demektir. Depth 1 çünkü yalnızca **en yeni** komut anlamlıdır;
  kuyruklanmış eski komutlar tehlikelidir.
- **`/arb/map` → TRANSIENT_LOCAL**: Geç başlayan bir planlayıcı, mevcut haritayı
  yine de almalı. Bu, "latched topic" davranışıdır.

Yayıncı ve abone QoS'leri uyumsuzsa **bağlantı hiç kurulmaz ve hata mesajı da
çıkmaz** — ROS 2'nin en çok zaman kaybettiren tuzağı budur.

---

## Mini görev cevabı bekleniyor

Aşağıdaki soruyu bir sonraki mesajında tartışacağız (cevabı şimdi vermiyorum):

> `normalizeAngle(+π)` neden `−π` döndürüyor, `+π` değil? Aralığı `[−π, π]`
> (kapalı) yerine `[−π, π)` (yarı açık) seçmenin testlerde ve karşılaştırmalarda
> ne faydası var?
