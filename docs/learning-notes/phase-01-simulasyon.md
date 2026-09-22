# Phase 1 — Simülasyon, Robot Modeli ve TF Ağacı

Bu fazda üç şey yaptık: robotu URDF ile tanımladık, ortamı YAML'dan SDF üreterek
oluşturduk, ve ikisini Gazebo'da birleştirip **gerçekten doğru çalıştığını
ölçtük**. Bu notun asıl değeri, yol boyunca bulduğumuz **dört gerçek bug**'da —
çünkü hepsi sessiz hatalardı.

---

## 1. Neden dünyayı YAML'dan üretiyoruz?

`configs/world.yaml` 100 satır. Ürettiği `arb_indoor.sdf` **1058 satır**.

Elle SDF yazsaydık:
- 27 model bloğu, her biri `<pose>`, `<collision>`, `<visual>`, `<inertial>` ile
- Bir duvarın pozunda tek bir işaret hatası → LiDAR ışınlarının kaçabileceği
  görünmez bir yarık → haritada açıklanamayan koridorlar → Phase 6'da saatlerce
  hata avı

Üretici (`scripts/generate_world.py`) ayrıca `--check` modu sunuyor: SDF elle
düzenlenip kaynağından ayrıştıysa CI bunu yakalar.

Bir de bonus: aynı `world.yaml`'ı Phase 5'te trajectory grafiklerinin altına
referans harita çizmek ve Phase 6'da üretilen occupancy grid'i **bilinen
gerçekle** karşılaştırmak için kullanacağız. Tek kaynak, üç tüketici.

### Duvar köşelerinde neden `length + thickness`?

```python
f"{length + thickness:.6f} {thickness:.6f} {height:.6f}"
```

İki duvar dik açıyla buluştuğunda, tam uzunlukta kutular köşede **diyagonal bir
yarık** bırakır. LiDAR ışını oradan sızar, duvarın arkasını görür, occupancy
grid'e hayalet serbest alan yazılır. Her segmenti bir kalınlık uzatmak köşeleri
üst üste bindirir.

---

## 2. URDF ve "tek kaynak" iddiasının gerçek olması

URDF, `configs/robot.yaml` ve `configs/sensors.yaml`'ı **parse zamanında
okuyor**:

```xml
<xacro:property name="cfg" value="${xacro.load_yaml('$(arg robot_config)')['robot']}"/>
<xacro:property name="wheel_r" value="${drive['wheel_radius_m']}"/>
```

Neden bu kadar önemli: simülasyondaki tekerlek yarıçapı ile Phase 3'teki
odometri denklemindeki yarıçap farklı olursa, robotun kat ettiğine *inandığı*
her mesafede **sabit oranlı bir ölçek hatası** olur. Bu hata trajectory
grafiğinde tam olarak "sensör drift'i" gibi görünür ve hangi katmandan geldiğini
anlamak çok zordur.

İddiayı test de zorluyor: `tests/integration/test_urdf_model.py` URDF'i üretip
her boyutu YAML ile karşılaştırıyor.

### Atalet tensörleri neden önemli?

Çoğu başlangıç URDF'inde her linkte `ixx=iyy=izz=0.001` görürsün. RViz'de model
doğru görünür, fizikte saçmalar: 1000 kat küçük dönme ataleti olan bir şasi ilk
tekerlek torkunda fırlar.

`inertia_macros.xacro` düzgün analitik tensörleri veriyor:

| Şekil | Tensör |
|---|---|
| Dikdörtgen prizma (x,y,z) | `I_xx = m/12·(y²+z²)`, döngüsel |
| Silindir (simetri ekseni etrafında) | `I = ½·m·r²` |
| Silindir (dik eksen) | `I = 1/12·m·(3r² + h²)` |
| Küre | `I = 2/5·m·r²` |

---

## 3. BULUNAN BUG #1 — Fixed joint lumping ve kaybolan `base_link`

URDF → SDF dönüşümünde Gazebo **fixed joint lumping** yapar: sabit eklemle
bağlı bir link, ebeveynine **eritilir** ve SDF'te ayrı link olarak kalmaz.
Pozlar korunur, yani fizik doğrudur — ama **link adı kaybolur**.

İlk versiyonda kök `base_footprint` idi. Sonuç:

```
SDF link'leri: base_footprint, left_wheel_link, right_wheel_link
```

`base_link` yok! Ve `OdometryPublisher` şöyle yapılandırılmıştı:

```xml
<robot_base_frame>base_link</robot_base_frame>
```

Bulamadığı için **hiçbir şey yayınlamaz**. Hata mesajı yok, uyarı yok — topic
sadece hiç oluşmaz. Phase 5'te "ground truth nerede?" diye bakarken keşfedilir.

**Çözüm:** kökü `base_link` yapmak. O zaman `base_footprint` eritilir ve
`base_link` hayatta kalır. Bu aynı zamanda REP-105'e daha uygun: REP-105 robot
çerçevesi olarak `base_link`'i adlandırır; `base_footprint` bir kolaylık
uzantısıdır ve tf2 dönüşümü zaten tersine çevirebilir.

**Test:** `test_sdf_conversion_keeps_base_link` bunu mekanik olarak kilitliyor.

---

## 4. BULUNAN BUG #2 — Spawn yüksekliği: robotu yere gömmek

```python
"-z", "0.05",   # "5 cm yukarıdan bırakalım"
```

`-z` **model origin**'ini konumlandırır. Bizim robotumuzda origin `base_link`,
yani şasi merkezi — modelin en alt noktası değil. Tekerlekler yere ancak
`base_link` zeminden `ground_clearance + height/2 = 0.10 m` yukarıdayken değer.

`0.05` yazmak robotu **5 cm yere gömüyordu**. Çözücü onu dışarı itiyor,
tekerlekler çıkarken sürtüyor, ve her koşunun ilk saniyeleri encoder arızası
gibi görünen kayma içeriyor.

Düzeltme: değeri config'ten hesapla.

```python
base_z = chassis["ground_clearance_m"] + chassis["height_m"] / 2.0
spawn_z = base_z + SPAWN_DROP_M
```

---

## 5. BULUNAN BUG #3 — Tek caster: robot burnunun üstünde duruyordu

Bu en sinsisiydi ve **sensör verisi üzerinden** kendini gösterdi.

### Belirti

LiDAR, boş koridorda ileri yönde **1.04 m'de hayali bir engel** raporluyordu.
Tarama-ile-kat-planı artığı: ortalama +0.18 m, std 0.32 m. Işınların %37'si
hiçbir yüzeye oturmuyordu.

İlk tepki: "LiDAR bozuk" veya "extrinsics yanlış". İkisi de değildi.

### Teşhis

Artıkların en kötüsü **bearing ≈ 0°** civarındaydı, yani tam ileri. Menzil
~1.04 m. LiDAR zeminden 0.25 m yukarıda. Yatay bir ışın düz zemine **asla**
değmez... eğer sensör eğik değilse.

```
range = h / sin(pitch)  →  sin(pitch) = 0.25/1.04 = 0.24  →  pitch ≈ 13.9°
```

Gazebo'ya gerçek 3B pozu sorduk:

```
Pose [XYZ] [-7.879780 -0.003236 0.099175]
     [RPY] [0.000000  0.203464 -2.737250]
                      ^^^^^^^^ = 11.66°
```

**Robot burnu aşağı 11.7° yatıktı.** Şasi ön kenarının üstünde duruyordu.

### Kök neden

- Tahrik tekerlekleri x = 0'da
- Şasi kütle merkezi x = 0'da
- Caster **sadece arkada**, x = −0.15'te

Destek poligonu {sol tekerlek, sağ tekerlek, arka caster}. Kütle merkezi bu
poligonun **tam ön kenarında**. Öne devrilme momentine direnen hiçbir şey yok.
Robot burnunun üstüne yatıyor ve orada kalıyor.

Devrilme açısı: `atan(ground_clearance / (chassis_length/2)) = atan(0.04/0.2)
= 11.31°` — ölçülen 11.66° ile (tekerlek sıkışması dahil) birebir.

### Çözüm

**İki caster**: ön (x = +0.15) ve arka (x = −0.15). Destek poligonu artık kütle
merkezini içeriyor.

### Sonuç — öncesi/sonrası

| Ölçüm | Tek caster | İki caster |
|---|---|---|
| Pitch | 0.2035 rad (11.7°) | **0.000000 rad** |
| z yüksekliği | 0.099 m | **0.100000 m** |
| Tarama artığı ortalaması | +0.1834 m | **−0.0007 m** |
| Tarama artığı std | 0.3204 m | **0.0149 m** |
| \|artık\| < 5 cm oranı | %62.8 | **%99.7** |
| Encoder yayı vs gerçek mesafe | %3.6 fark | **%0.0 fark** |

Son satır kritik: Phase 3'ün tamamı "tekerlek yayı = kat edilen mesafe"
ilişkisine dayanıyor. %3.6'lık sistematik hata ile dead reckoning baştan bozuk
olurdu.

**Ders:** Sensör verisi tuhaf göründüğünde, hata sensörde değil **modelde**
olabilir. Ve "sensör bozuk" ile "robot yatık" aynı veriyi üretir.

---

## 6. BULUNAN BUG #4 — LiDAR açı adımı: tekrarlanan ışın

Gazebo şöyle hesaplıyor:

```
angle_increment = (angle_max - angle_min) / (samples - 1)
```

`angle_min = −π`, `angle_max = +π`, `samples = 360` verince:

```
increment = 2π/359 = 0.01750191 rad = 1.0028°
```

Ve **ışın 0 (−π) ile ışın 359 (+π) aynı yöne bakıyor** — bir ölçüm boşa
gidiyor, hüzme aralığı da tam 1° değil.

Düzeltme: span'i bir adım kısalt.

```
angle_max = π − 2π/360   →   increment = 2π·(359/360)/359 = 2π/360 = tam 1°
```

Doğrulama: artık analizi bearing değerlerini tam tamsayı basıyor
(`−88.00°`, `78.00°`, `98.00°`) ve kullanılan ışın 347/360 → **360/360** oldu.

---

## 7. TF ağacı — `map → odom → base_link` neden böyle?

```
map ──(EKF düzeltmesi: yavaş, SÜREKSİZ)──▶ odom ──(tekerlek odometrisi: hızlı, PÜRÜZSÜZ)──▶ base_link
                                                                                              ├─▶ imu_link
                                                                                              ├─▶ lidar_link
                                                                                              └─▶ base_footprint
```

| Dönüşüm | Özellik | Kim kullanır |
|---|---|---|
| `odom → base_link` | Pürüzsüz, sürekli, **sınırsız drift** | Kontrolcü (sıçrama robotu sarsar) |
| `map → odom` | Global tutarlı, **süreksiz** (scan match düzeltmesinde sıçrar) | Planlayıcı (global tutarlılık şart) |

Düzeltmeyi `odom → base_link`'e yazmak her iki özelliği de yok eder. ROS
navigation'ın en çok yanlış anlaşılan kısmı budur.

Phase 1'de henüz EKF yok, o yüzden `map → odom` yayınlanmıyor. Phase 4'te
gelecek.

**Şu an doğrulanmış statik TF:**

```
base_link → base_footprint  (0, 0, −0.10)
base_link → front_caster_link  (+0.15, 0, −0.08)
base_link → rear_caster_link   (−0.15, 0, −0.08)
base_link → imu_link        (0, 0, +0.08)
base_link → lidar_link      (+0.14, 0, +0.15)
```

Hepsi `configs/robot.yaml` ile birebir.

---

## 8. Topic grafiği

```
            ┌──────────────────────┐
            │   gz sim (Gazebo)    │
            │  fizik + sensörler   │
            └──────────┬───────────┘
                       │  gz-transport (DDS DEĞİL!)
                       │
            ┌──────────▼───────────┐
            │   ros_gz_bridge      │   ← configs: ros_gz_bridge.yaml
            └──────────┬───────────┘
                       │  ROS 2 DDS
   ┌───────────────────┼────────────────────────┬──────────────────┐
   │                   │                        │                  │
/clock          /imu/data_raw            /joint_states         /scan_raw
1000 Hz            200 Hz                    50 Hz               10 Hz
   │                                                                │
   │            /arb/ground_truth/odom (100 Hz)  ← SADECE DEĞERLENDİRME
   │                                                                │
   └──────────────► use_sim_time=true olan tüm node'lar ◄───────────┘

/cmd_vel  ──(ROS → GZ)──▶  DiffDrive plugin  ──▶ tekerlek torkları

/robot_description ──▶ robot_state_publisher ──▶ /tf, /tf_static
```

### Köprü hakkında bilinmesi gereken

Gazebo ve ROS 2 **iki ayrı middleware**: `gz-transport` ve DDS. Aralarında
hiçbir şey kendiliğinden geçmez. URDF'te mükemmel yapılandırılmış bir sensör,
`ros_gz_bridge.yaml`'a yazılmazsa Gazebo içinde mutlu mutlu yayın yapar ve
ROS'tan **tamamen görünmez** olur. Bu, bozuk bir sensörle birebir aynı görünür.

### `/clock` neden kritik?

Tüm node'lar `use_sim_time=true` ile çalışıyor. `/clock` köprülenmezse tüm graf
zaman sıfırda takılır, TF sorguları "requested time is in the future" ile
başarısız olur ve hiçbir şey çalışmaz.

---

## 9. Doğrulama felsefesi: topic'in var olması kanıt değildir

`ros2 topic list` bir topic'i gösteriyorsa bu sadece **birinin o adı kaydettiği**
anlamına gelir. Bu yüzden `scripts/verify_simulation.py` içeriği kontrol ediyor:

- yayın hızı (config ile ±%15)
- `frame_id` doğru mu
- IMU gürültüsü gerçekten enjekte ediliyor mu (std > 0)
- LiDAR ışın sayısı, açı aralığı, hüzme adımı
- menziller `[range_min, range_max]` içinde mi
- robot komut edildiğinde gerçekten hareket ediyor mu
- **tekerlek yayı ile gerçek mesafe uyuşuyor mu**

Son madde Phase 3'ün ön koşulu.

### QoS tuzağı

```python
SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, ...)
```

Köprü sensör verisini `BEST_EFFORT` yayınlıyor. `RELIABLE` bir abone **QoS
uyumsuz** olur ve **hiçbir mesaj almaz** — üstelik hata da vermez. "Topic var
ama echo boş" şikayetinin bir numaralı sebebi budur.

### Sürüş testi ortama uyarlanmalı

İlk versiyon sabit 4 saniye sürüyordu. Robot spawn'da (−8.5, 0), koridorda
(−7.0, 0)'da r=0.25 bir sütun var. Robot ön yüzünden sütuna mesafe tam **1.05
m** — ve test tam 1.050 m ölçtü. Robot sütuna çarpmıştı; tekerlekler dönmeye
devam etti ve **%15 kayma** gibi göründü.

Düzeltme: sürmeden önce LiDAR'dan ileri açıklığı oku, komutu ona göre boyutlandır.

**Ders:** açıklanamayan bir ölçüm gördüğünde, önce **ölçüm düzeneğini** sorgula.

---

## 10. Beam indeksleme tuzağı

```python
forward_idx = int(round(-scan.angle_min / scan.angle_increment))
```

"İleri = indeks 0" varsaymak yaygın bir hatadır. `angle_min = −π` olan bir
taramada **indeks 0 geriye bakar**. İleri ışın `−angle_min / angle_increment`
indeksindedir. Phase 6 ve 8'de bu sürekli lazım olacak.

---

## Mini görev

Bir sonraki mesajında tartışacağız:

> `configs/robot.yaml` içinde `casters.x_offsets_m` değerini `[0.150, -0.150]`
> yerine `[0.050, -0.050]` yap, yeniden derle, simülasyonu başlat ve
> `scripts/verify_simulation.py` ile `gz model -m arb_diffdrive -p` çıktısına
> bak.
>
> Robot hâlâ düz duruyor mu? Sürüş testinde ne değişiyor? **Neden?**
>
> İpucu: destek poligonunun boyutu ile devrilmeye direnen momentin ilişkisini
> düşün. Kütle merkezi hâlâ poligonun içinde ama...
