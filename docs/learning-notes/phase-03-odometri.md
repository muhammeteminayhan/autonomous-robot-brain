# Phase 3 — Diferansiyel Sürüş Odometrisi

Bu faz iki şey üretti: çalışan bir dead reckoning ve **onun ne kadar hızlı
yanlışlandığının ölçümü**. İkincisi daha değerli, çünkü Phase 4'teki EKF'in
neden var olduğunun kanıtı o.

---

## 1. Kinematik — nereden geliyor?

İki tekerlek, yarıçap `r`, aralarında `L` mesafe, açısal hızları `ω_l`, `ω_r`.
Temas noktaları kaymadan yuvarlanıyor, yani her tekerleğin yere göre hızı
`v = r·ω`.

Gövde rijit, iki temas noktası tek bir dönme merkezini paylaşıyor. Gövde
twist'ini (v, ω) orta noktada yazarsak:

```
v_r = v + ω·L/2
v_l = v − ω·L/2
```

Çiftini çözünce **ileri kinematik**:

```
v = (v_r + v_l)/2 = r·(ω_r + ω_l)/2
ω = (v_r − v_l)/L = r·(ω_r − ω_l)/L
```

Dikkat: tekerlekler sana **sadece v ve ω** söyler. Pozisyon hakkında hiçbir şey
söylemez. Pozisyon ancak integrasyondan sonra vardır ve bir adımda yapılan her
hata **sonsuza kadar taşınır**.

### Holonomik olmama

Diferansiyel sürüş yana gidemez. Gövde çerçevesinde hız daima `(v, 0, ω)`.
Dünya çerçevesinde:

```
ẋ = v·cos(θ)
ẏ = v·sin(θ)
θ̇ = ω
```

---

## 2. İntegrasyon — naif Euler neden yanlış?

Doğal görünen ayrıklaştırma:

```cpp
x += v * cos(theta) * dt;   // YANLIŞ, üstelik taraflı şekilde yanlış
```

Bu, yönün adım boyunca **sabit** olduğunu varsayar. Dönerken her zaman köşe
keser: yayın kirişini **başlangıç teğetiyle** integre eder. Hata ortalamada
sıfırlanmaz çünkü dönüşe göre hep aynı yöne bakar — daire çizen bir robot her
adımda biraz içeri spiralleşir.

### Tam çözüm

`v` ve `ω` adım boyunca sabitse yol tam bir **daire yayı**dır:

```
x += (v/ω)·[sin(θ + ω·dt) − sin(θ)]
y += −(v/ω)·[cos(θ + ω·dt) − cos(θ)]
```

`a = ω·dt` koyup toplam-çarpım özdeşliklerini uygularsak:

```
sin(θ+a) − sin(θ) =  2·cos(θ + a/2)·sin(a/2)
cos(θ+a) − cos(θ) = −2·sin(θ + a/2)·sin(a/2)
```

ve `ω = 0`'da tekilliği **olmayan** biçim çıkar:

```
x     += v·dt · sinc(a/2) · cos(θ + a/2)
y     += v·dt · sinc(a/2) · sin(θ + a/2)
θ     += a
```

`sinc(u) = sin(u)/u`, `sinc(0) = 1`.

Üç şey dikkat çekici:

1. **`ω = 0`'da özel durum yok.** Klasik `if (fabs(omega) < 1e-6)` dalı
   yalnızca `(v/ω)` biçimi kullanıldığı için vardır. Özdeşlik onu gereksiz
   kılar.
2. **Kullanılan yön adımın ORTA NOKTASI** (`θ + a/2`), başlangıç değil. Tek
   başına bu, birinci mertebe ile ikinci mertebe doğruluk arasındaki farktır.
3. **`sinc(a/2) ≤ 1` her zaman.** Bu, kirişin yaya oranıdır: düz çizgi yer
   değiştirmesi gerçekten de tekerleklerin yuvarlandığı mesafeden kısadır.

Bu aslında SE(2)'nin **üstel haritası**dır (exponential map) — bu yüzden
"yaklaşık iyi" değil, **tam**.

> Test kanıtı: `CircularArcClosesOnItself` — tam bir daire çizen robot
> başlangıç noktasına sayısal hassasiyette geri dönüyor.
> `ArcAccuracyIsIndependentOfStepSize` — adım boyutunu 50 kat küçültmek
> sonucu değiştirmiyor, çünkü iyileştirilecek bir şey kalmamış.

### `sinc` neden Taylor serisiyle?

`u = 1e-8` civarında `sin(u)` içindeki çıkarma zaten anlamlı basamakların
çoğunu atmıştır; `sin(u)/u` gözle görülür şekilde 1'den sapar.
`1 − u²/6 + u⁴/120` serisi `|u| < 1e-4` için hem hızlı hem hassas.

---

## 3. Kovaryans — belirsizlik nasıl büyür?

```
P ← F_x · P · F_xᵀ + F_u · Q · F_uᵀ
```

`Q` tekerlek-yay uzayında **köşegen** ve **|yay| ile orantılı**: kayan bir
tekerlek, ne kadar döndüyse o oranda kayar. Yani **varyans mesafeyle lineer**,
standart sapma `√mesafe` ile büyür.

İki tekerlek **bağımsız** kabul edilir — heading belirsizliğinin büyümesinin
tek sebebi budur. Mükemmel korelasyonlu tekerlek hataları `(s_r − s_l)`
farkında birbirini götürürdü.

### Jacobian tuzağı: tam integre edip yaklaşık türev alma

İlk yazdığım `F_u` klasik ders kitabı formülüydü. Ama ders kitabının
güncellemesi orta-nokta **yaklaşımı**dır:

```
x' = x + Δs·cos(θ + Δθ/2)      ← sinc yok
```

Benim güncellemem ise `sinc(a)` çarpanı içeriyor ve `a`, tekerlek yaylarına
bağlı. Yani `∂x'/∂s_r` **sinc'in türevini de** içermeli.

Sonlu fark testi bunu %0.045'lik sistematik bir uyuşmazlık olarak yakaladı.
Küçük görünüyor ama **sistematik** — ve EKF'i sessizce aşırı özgüvenli yapan
tam olarak bu tür şeylerdir.

Tam türev:

```
a = Δθ/2,  k = sinc(a),  k' = sinc'(a)
∂Δs/∂s_r = 1/2,  ∂a/∂s_r = 1/(2L)

∂x'/∂s_r = ½·k·cos(θ+a) + Δs·[k'·cos(θ+a) − k·sin(θ+a)]/(2L)
```

> **Kural: tam integre ediyorsan, tam türev al.** `F`, `f`'e uymazsa filtre
> tutarsız olur.

---

## 4. BULUNAN BUG #1 — Deney tasarımı: robot sütuna çarptı

İlk drift ölçümü şunu verdi: **%34.7 hata, 172.88° maksimum heading hatası.**

Sebep: spawn pozu koridorda, 1.5 m ileride bir sütun var. Açık-döngü rota
doğrudan sütuna sürdü. Tekerlekler dönmeye devam etti, encoder'lar saydı,
odometri **hiç olmayan hareketi** integre etti.

Bu bir estimator ölçümü değil, bir **çarpışma** ölçümüydü.

İki düzeltme:
1. Kat planı üzerinde **en geniş engelsiz daireyi** tarayıp (3.0, −3.5),
   her yöne 1.93 m, oraya `odometry_test` pozu eklendi.
2. Değerlendirme scriptine **LiDAR tabanlı iptal guard'ı**: ileri açıklık
   0.45 m'nin altına düşerse koşu durur ve **ABORTED** der.

> Sessizce bir çarpışmayı ölçen bir değerlendirme, hiç çalışmayandan kötüdür.

---

## 5. BULUNAN BUG #2 — Etkin iz genişliği geometrik değil

Temiz koşudan sonra sayılar hâlâ tuhaftı:

```
kat edilen mesafe        10.03 m
son pozisyon hatası       0.989 m  (%9.86)
son heading hatası      −72.36°
odometri loop closure     0.049 m   ← odometri kapattığını sanıyor!
```

Odometri kendi içinde tutarlıydı ve tamamen yanlıştı. Veriyi ayırınca:

| Ölçüm | Odometri | Gerçek | Oran |
|---|---|---|---|
| Kat edilen mesafe | 10.044 m | 10.031 m | **1.0013** ✓ |
| Toplam dönüş | 722.34° | 794.71° | **1.1002** ✗ |
| Hata dönüşlerde | — | — | +73.05° |
| Hata düzlerde | — | — | −0.68° |

Mesafe kusursuz, dönüş %10 yanlış. Tüm hata **dönüşlerde** birikiyor.

### Neden?

`ω = r·(Δθ_r − Δθ_l)/L` — dönüş sadece `L`'ye bağlı. Etkin `L`:

```
L_eff = 0.350 × 0.9010 = 0.3154 m
```

URDF'teki 0.350 m **yanlış değil** — tekerlek merkez düzlemleri gerçekten
0.350 m arayla. Ama **silindir bir tekerlek yere bir NOKTADA değil, bir
ÇİZGİDE** değer, ve robotu döndüren traksiyon merkez düzleminde etki etmez.

0.3154 m, tekerlek **iç kenarları** arası mesafeye (0.350 − 0.035 = 0.315 m)
milimetre hassasiyetinde eşit.

### Kalibrasyon

`scripts/calibrate_odometry.py`: her iki yönde 3'er tam tur, ground truth ile
karşılaştır.

```
rotation ratio (odom/true)   0.9010
direction asymmetry          0.00 %     ← tek bir L hatayı tam yakalıyor
distance ratio (true/odom)   0.9995

wheel_separation_m  0.3500 -> 0.3154   (-9.90 %)
wheel_radius_m      0.06000 -> 0.05997 (-0.05 %)
```

**Gerçek donanımda** ground truth yoktur; standart yöntem **UMBmark**
(Borenstein & Feng, 1996): 4×4 m kare, saat yönünde ve tersi, bitiş hatalarını
şeritmetre ile ölç, aynı iki bilinmeyeni çöz. Aritmetik aynı, yalnızca
gerçeğin kaynağı farklı.

### Sonuç

| Metrik | Kalibrasyonsuz | Kalibre |
|---|---|---|
| Son pozisyon hatası | 0.989 m (%9.86) | **0.095 m (%0.94)** |
| Son heading hatası | −72.36° | **6.96°** |
| RMS pozisyon hatası | 0.540 m | **0.055 m** |

**10 kat iyileşme, tek bir sayı değiştirerek.**

> **Sistematik hata gürültü değildir.** Hiçbir Kalman filtresi, kendisine
> söylenmemiş bir bias'ı kaldırmaz. Phase 4'te bunu "tune ederek" düzeltmeye
> çalışsaydım asla başaramazdım.

---

## 6. Kovaryans ayarı — aşırı muhafazakârlık da zararlı

Kalibrasyondan sonra raporlanan `1σ = 0.737 m` iken gerçek hata `0.091 m`'ydi.
8 kat muhafazakâr.

Zararsız değil: Phase 4'te Kalman kazancı, süreç ve ölçüm belirsizliklerinin
**oranıyla** belirlenir. 8 kat kötü olduğunu iddia eden bir tahmin, filtreyi iyi
odometriyi atıp her ölçümün peşinden koşmaya iter — sonuç bariz yanlış değil,
**titrek** bir kestirimdir.

`σ ∝ √k` olduğundan `k = 2e-4` (onda biri) raporlanan yayılımı üçte bire
indiriyor: 10 m sonra ~0.23 m ve 12.7°, yani gözlenen hatanın 2-3 katı.
Kasten hâlâ muhafazakâr tarafta: **aşırı özgüvenli filtre ıraksar, biraz
muhafazakâr olan sadece yavaş yakınsar.**

---

## 7. `map → odom → base_link` — neden bu node sadece ikincisini yayınlar?

```
map  ──(düzeltme, yavaş, SÜREKSİZ)──▶ odom ──(dead reckoning, hızlı, PÜRÜZSÜZ)──▶ base_link
```

- Dead reckoning **sürekli ve türevlenebilir** ama sınırsız kayar.
- Düzeltilmiş global kestirim **sınırlı** ama düzeltme geldiğinde **sıçrar**.

Kontrolcü birinciye muhtaç: takip ettiği dönüşümdeki bir sıçrama, hata
sinyalinde basamak olarak görünür ve robotu sarsar. Planlayıcı ikinciye
muhtaç: kayan bir çerçevede çizilen plan yavaşça dünyayla uyuşmaz olur.

Düzeltmeyi `map → odom`'da tutmak ikisini aynı anda sağlar. `odom → base_link`
içine yazmak **ikisini de yok eder** — ROS navigation'ı bozmanın bir numaralı
yolu.

---

## 8. Mutasyon testi — 8/8

| # | Mutasyon | İlk tur | Son tur |
|---|---|---|---|
| M1 | Naif Euler (başlangıç yönü) | ✅ | ✅ |
| M2 | `sinc` çarpanı kaldırıldı | ✅ | ✅ |
| M3 | `Δθ` işareti ters | ✅ | ✅ |
| M4 | `v`'de ½ çarpanı yok | ✅ | ✅ |
| M5 | Kovaryans Jacobian'ı başlangıç yönünü kullanıyor | ❌ | ✅ |
| M6 | Kovaryansta `F_u` terimi yok | ✅ | ✅ |
| M7 | `sinc` türevi ihmal edildi | — | ✅ |
| M8 | Kovaryans simetrikleştirme kaldırıldı | ❌ | ✅ |

**M5 neden kaçtı:** kovaryans testlerim iz, simetri ve PSD bakıyordu — hepsi
değişmemişti, yalnızca **belirsizlik elipsinin şekli** yanlıştı. Çözüm:
Jacobian'ları açığa çıkarıp **merkezi sonlu farkla** doğrulamak. Bu testler M7'yi
de ortaya çıkardı.

**M8 neden kaçtı:** `1e-15` toleransı kullanıyordum. Kod **tam** simetri garanti
ediyor, o yüzden testi tam eşitliğe (`EXPECT_EQ(..., 0.0)`) sıkılaştırdım. Bir
tolerans, "yapı gereği simetrik" ile "şimdilik şanslı" arasını ayıramaz.

---

## Mini görev

> `ros2_ws/src/arb_localization/config/localization.yaml` içindeki
> `wheel_separation_m`'i kalibre değer `0.3154` yerine geometrik `0.350` yap,
> derle, `scripts/evaluate_odometry.py` çalıştır.
>
> 1. Pozisyon hatası kaça çıkıyor?
> 2. **Kat edilen mesafe** değişiyor mu? Neden?
> 3. `tests/integration/test_config_consistency.py` ne diyor?
> 4. Bu hatayı Phase 4'te `k_right`/`k_left`'i büyüterek "tune ederek"
>    düzeltebilir miydin? **Neden olmaz?**
