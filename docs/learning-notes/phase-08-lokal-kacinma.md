# Phase 8 — DWA: Pozisyon değil, HIZ uzayında arama

## 1. Neden hız uzayı?

Global planlayıcı **pozisyon** arar ve robotun yapamayacağı bir yol üretebilir:
dik açılı dönüş, anlık duruş, ters yön. Bu onun **hatası değil** — işi rota
bulmak, o rotayı sürmek değil.

DWA bunun yerine **hız** arar. Her aday, robotun bir kontrol periyodunda ivme
limitleri dahilinde gerçekten ulaşabileceği bir `(v, ω)` çiftidir. Yani
"uygulanamaz plan" diye bir şey **tanım gereği** yok.

```
Dinamik pencere = [v − a·dt, v + a·dt] × [ω − α·dt, ω + α·dt]
                  ∩ [aktüatör limitleri]
```

Bu pencere içindeki her noktayı örnekle (9 × 21 = 189 aday), her birini sabit
twist ile 2 s ileri simüle et, puanla, en iyisini seç. Saniyede 20 kez.

## 2. Dört terim ve her birinin tek başına yaptığı aptallık

| Terim | Ne ister | Tek başına |
|---|---|---|
| heading | rollout sonunda carrot'a bakmak | **duvara sürer** |
| clearance | engellerden uzak durmak | **en geniş boşluğa park eder, hiç çıkmaz** |
| velocity | hızlı gitmek | **gördüğü ilk şeye tam gaz girer** |
| path | rollout sonunda carrot'a yakın olmak | — |

Tasarım tam olarak bu gerilimdir. Ağırlıkların anlamlı olması için her terim
önce **normalize** edilir (o döngüdeki min–max'a göre); yoksa `heading_weight`
gerçekte neyi ağırlıklandırdığın bilinmez.

## 3. Admissible velocity — "ufuk temiz olsun" DEĞİL

Naif güvenlik kuralı: *rollout bir şeye çarpıyorsa adayı reddet.* Bu çok katı.
0.45 m boşluğu olan, 0.3 m/s giden robot **hiçbir** kabul edilebilir komut
bulamaz ve olduğu yerde durur.

Doğru koşul: robot gördüğü şeyden önce **durabilmeli**:

```
v ≤ √(2 · d · a_max)
```

Klasik kinematik: `v² = 2·a·d`. Bu, "hiç çarpma" değil "zamanında durabil"
koşulu — ve aradaki fark, çalışan bir robotla donmuş bir robot arasındaki fark.

Aynı koşul artık **hedefe** de uygulanıyor. Amaç fonksiyonunda hiçbir terim
yolun bittiğini bilmiyor; bu olmadan robot limitte gidip varış kontrolü
tetikleyene kadar devam eder ve **zınk diye durur**. Hedefi aşıp "dur" emri
almak, varmak değildir.

### Mesafe hep YOL BOYUNCA ölçülür

Geri dönen bir yolda (tur, duvarın etrafındaki L) düz çizgi mesafesi kısalırken
sürülecek metreler durur. Düz çizgi kullanılırsa robot tüm turu sürünerek atar.

## 4. Çarpışma kontrolü zamanla değil MESAFEYLE adımlanır

0.66 m/s'de 0.1 s adımla rollout 6.6 cm atlar — ve çözünürlük **hıza bağlı**
hale gelir. Böyle ölçüldüğünde planlayıcı 0.20 m'lik boşluğu **0.265 m** diye
raporladı. Artık yay uzunluğuyla adımlanıyor ve **aşağı yuvarlanıyor**.

## 5. Carrot hatası — bu projede iki kez, bağımsız olarak

DWA, path terimini **carrot**'a göre puanlar: yol üzerinde sabit bir yay
uzunluğu ileri.

**En yakın noktayı puanlamak yerinde durmayı ödüllendirir** — yol robottan
başlar, robot zaten üzerindedir, hareket etmemek optimaldir. Ölçüldü: en iyi
puanlı komut her döngüde pencerenin izin verdiği **en yavaş** komuttu.

Ama carrot'u **yolun ilk noktasından** yürümek daha kötü:

```
YANLIŞ:  carrot = path[0]'dan lookahead metre ileri
DOĞRU:   carrot = ROBOTUN PROJEKSİYONUNDAN lookahead metre ileri
```

İkisi sadece `t = 0` anında aynıdır. Sonra carrot ilerlemez, robot ona varır,
geçer ve **arkasındaki** bir noktaya nişan alır.

Ölçülen: **140 saniyede 15.60 m'lik parkurun 1.99 m'si.**

Aynı hata PID takipçisinde de bağımsız olarak vardı. İkisinde de yorum satırı
"robottan itibaren yürü" diyordu. İkisi de testlerden geçiyordu — çünkü **her
test yolu robotun etrafında yeniden kuruyordu**, ve robotla birlikte kayan bir
yolda `path[0]` ile projeksiyon sonsuza kadar aynı noktadır.

Global planlayıcı da tam olarak bunu yapar: saniyede bir, robotun konumundan
yeniden planlar. Yani hata **replanner'ın arkasına saklandı** ve ancak yol sabit
tutulduğunda ortaya çıktı.

Çözüm: geometri `arb_core/path_geometry.hpp`'ye taşındı — tek implementasyon,
tek test seti, ve planlayıcıyı kontrolcüye bağımlı yapmayan bir bağımlılık yönü.

## 6. Ölçülen

15.60 m'lik parkur, 3 köşe, Oda B:

| | DWA |
|---|---|
| parkuru bitirdi | evet |
| süre | 32.0 s |
| cross-track RMSE (kalıcı) | 0.085 m |
| en kötü kalıcı hata (bir köşe) | 0.222 m |
| yerleşme süresi (0.6 m basamak) | 3.55 s |
| aşım | **%0.00** |
| komut pürüzsüzlüğü (açısal) | **0.234 rad/s²** |
| direksiyon yön değişimi | **2** |
| çarpışmasız | evet |

Uçtan uca: 16.8 m, 36.5 s, koridordan geçip 1.5 m'lik kapıdan Oda E'ye,
çarpışmasız, en yakın yaklaşma 0.40 m.

## Mini görev

> 1. `planning.yaml`'da `clearance_weight`'i 0.7 → 0.0 yap ve
>    `scripts/evaluate_navigation.py --mode goal --goal goal_room_e` çalıştır.
>    `min_lidar_range_m` ne oluyor? **Neden** hâlâ çarpışmıyor?
> 2. `simulation_time`'ı 2.0 → 0.5 s yap. Robot hangi noktada tıkanıyor?
>    İpucu: 0.7 m/s'de 0.5 s ne kadar yol demek, ve durma mesafesi
>    `v²/(2a)` ne kadar?
> 3. `enforce_stopping_distance: false` yap. Koridor sütununun yanından
>    geçerken ne değişiyor?
