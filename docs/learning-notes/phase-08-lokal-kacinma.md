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

## 7. Sonradan eklenen: hareketli engeller

Phase 11, DWA'nın koridoru geçen bir engele **3 koşunun 3'ünde de** çarptığını
ölçtü. Planlayıcı dikkatsiz değildi — **yanlış soruyu** cevaplıyordu.

DWA, yörüngeyi **şu anda gördüğü** engel konumları üzerinden ileri sarıyor.
Geçen bir engel, rollout hesaplandığı anda süpürülen hacmin içinde **değil**;
oraya **sonradan** yürüyor. Her 50 ms'lik çevrimde planlayıcı, yolun —
*engelin bulunduğu yerden* — temiz olduğunu doğru biçimde yeniden tespit
ediyordu.

### Lazer taramasında hız yoktur

Düzeltmek için engel hızı lazım, ama tarama hız içermez: ardışık karelerde
**hiçbir karşılıklılık (correspondence) olmayan** bir nokta kümesidir. O
karşılıklılığı üretmek gerekiyor.

`obstacle_tracker` her taramayı **kümelere** ayırıyor, kümeleri önceki
çevrimin izleriyle **en yakın merkez** kuralıyla eşleştiriyor ve türevliyor.

### Atlanması kolay, atlanması ölümcül adım: ego-hareket telafisi

```
Tarama ROBOT çerçevesinde gelir, ve robot hareket ediyor.
  -> Duran bir duvar, robotun kendi hızıyla yanından geçiyormuş gibi görünür.
  -> Merkezleri ORADA farkını alırsan: her durağan nesne -v hızla hareket ediyor.
  -> Planlayıcı tüm dünyanın üstüne geldiğini öngörür ve HİÇ hareket etmez.
```

Bu yüzden merkezler, fark alınmadan **önce sabit çerçeveye** çevriliyor; elde
edilen dünya hızı da planlayıcı için robot eksenlerine geri döndürülüyor.
`StaticObstaclesMeasureZeroVelocityWhileTheRobotMoves` testi tam olarak bunu
doğruluyor.

İki tuzak daha:
- **Duvarlar nesne değildir.** Duvarın merkezi, robot geçerken duvar boyunca
  kayar ve saf geometriden bir hız **uydurur**. 1.0 m'den geniş kümeler yapı
  sayılıp izlenmiyor.
- **İnanılmaz hız, hızlı engel değil, eşleştirme hatasıdır.** Birbirine
  eşleşen iki farklı nesne devasa bir görünür hız üretir; buna inanmak
  planlayıcıyı **var olmayan** bir şeyden kaçırır.

### "Sabit çerçeve" yetmez: HANGİ sabit çerçeve?

Kolayca gözden kaçan, çünkü **her şeyi doğru yapmış gibi** görünen bir
ayrım. Yukarıdaki kural "sabit çerçeveye çevir" diyor. `map` sabit bir
çerçeve. Üstelik *doğru* olanı — kayma biriktirmeyen tek çerçeve o. Yine de
yanlış cevap.

```
REP-105: map -> odom SÜREKSİZDİR, hem de BİLEREK.
  Lokalizasyon düzeltmelerini oraya koyar -- işi bu.
  Her kabul edilen scan match, map->odom'u sıçratır.

İzleyici ise merkez farkını 0.1 s üzerinden alıyor.
  -> Ona lazım olan "ortalamada doğru" çerçeve değil,
     "0.1 saniyede PÜRÜZSÜZ" çerçeve.

2 cm'lik bir düzeltme, 10 Hz'de = 0.2 m/s görünür hız.
  Ve bu hız TÜM DURAĞAN DÜNYAYA aynı anda uygulanır.
```

`odom` kayar, ama buradaki kayma zararsız: bir çevrime göre yavaş, ve hız
hemen robot eksenlerine geri döndürülüyor. Yani **doğruluk değil,
pürüzsüzlük** gerekiyor. Bunlar farklı şeyler ve bu proje bunu zor yoldan
öğretti.

Peki ölçüm ne dedi? **Çerçeve asıl sorun değilmiş.**

İlk koşum muhteşem görünüyordu: hayalet ortalaması 0.8, hedefe 39.6 s'de
varış. Tabloyu yazmaya başlamıştım. Sonra aynı yapılandırmayı tekrar
koşturdum: **ortalama 3.84, en çok 9 hayalet**, 180 s zaman aşımı.

```
Aynı kod. Aynı tohum. Aynı komut.
  1. koşum: 0.8 hayalet, vardı
  2. koşum: 3.84 hayalet, varamadı

-> n=1 ile "düzeldi" demek ölçüm değil, TEMENNİdir.
```

Çerçeve değişikliği doğru ama yetersizdi. Asıl suçluyu bir test değil, bir
**log satırı** buldu: her izin konumunu, hızını, genişliğini ve yaşını
saniyede bir yazdırınca örüntü ortaya çıktı.

```
[-1.70,0.71 v=0.31 ext=0.43 hits=218]
[-2.22,0.68 v=0.32 ext=0.45 hits=238]
[-2.70,0.60 v=0.26 ext=0.31 hits=258]
```

Robotun arkasına doğru, robotun kendi hızıyla, yüzlerce çevrim yaşayan,
0.3–0.45 m genişliğinde izler. Dünyada hiçbir şey böyle davranmaz. Ama
**kayan bir pencere** aynen böyle davranır.

### Genişlik, "küçük nesne" ile "büyük şeyin küçük parçası"nı ayıramaz

`DoesNotTrackWallsAsObjects` geçiyordu, çünkü oradaki duvar **baştan sona
görünür**: küme 4 m, yapı filtresi reddediyor. Gerçek duvarlar baştan sona
görünmez. 0.3 m'lik bir kapı aralığından duvar, 1.0 m eşiğinin çok altında
bir parça döndürür — filtrenin reddedecek bir şeyi yoktur. Ve robot
ilerledikçe görünür pencere duvar boyunca kayar, merkezi de peşinden.

Ayrım genişlikte değil, **kümeyi neyin sınırladığında**:

| | sınırlayan | |
|---|---|---|
| açıkta duran nesne | daha uzaktaki dönüşler ya da hiç dönüş yok | gerçek kenar |
| aralıktan bakış | **iki yanda da** daha yakın dönüşler | aralığın kendisi |

Kural: bir kümenin iki açısal komşusu da ondan `aperture_margin_m` kadar
yakınsa, küme atılır. **İkisi de**, "biri" değil — köşeden ya da kapı
kasasından tek yanı kırpılmış nesne istisna değil kuraldır, ve yarı örtülü
engellere inanmayı bırakan bir kaçınma algoritması, hiç filtresi olmayandan
**daha kötüdür**: attığı şeyler tam da çarpmak üzere olduklarıdır.

Bunu `AnObjectHiddenDownOneSideIsStillAnObject` koruyor — ve yazılması
şarttı: `&&` yerine `||` koyan mutasyon, ilk test setinden **fark edilmeden
geçti**. Yani kuralın yarısı test edilmiyormuş.

> **Sentetik tarama tuzağı.** Kapı testinin ilk hâli yanlış sebeple
> düşüyordu. Yelpazem ±1.0 radyandı; bu, kestiği duvarın ucunda 0.41 m'lik
> bir "nesne" bırakıyordu — komşuları daha uzak olduğu için kurala takılmayan,
> tamamen **testin kendi kenarından** doğan bir yapaylık. Doğrusu: kapalı oda
> + gerçek ışın izleyici + tam 360°. Sentetik tarama üretirken dünyanın
> değil, taramanın sınırlarını ölçme riski her zaman vardır.

> **Mülakat sorusu olarak:** "`map` ve `odom` arasındaki fark nedir?" sorusuna
> herkes "biri global, biri lokal" der. Asıl cevap: **`odom` pürüzsüz ama
> kayar, `map` doğru ama sıçrar** — ve hangisini seçeceğin, doğruluk mu
> süreklilik mi istediğine bağlıdır. Türev alan her şey (hız kestirimi,
> izleme) `odom` ister; konum soran her şey (hedef, harita) `map` ister.

### İki farklı kural, çünkü fizik farklı

| | durağan engel | hareketli engel |
|---|---|---|
| soru | *önünde durabilir miyim?* | *o oradayken ben de orada mı olacağım?* |
| kural | `v ≤ √(2·d·a_max)` | ufuk içinde temas öngörülüyorsa **reddet** |

Durma kuralı, üstüne gelen bir şey için **fazla hoşgörülü**: ölçüldü, geçen
bir engele karşı ancak **0.12 m** ayrımda bağlıyor — o noktada çarpışma zaten
kaçınılmaz. **Durmak işe yaramıyor, engel yine de geliyor.**

Katı kural ise duran bir şey için **fazla muhafazakâr**: "tüm ufku temiz tut"
demek, 0.45 m boşluğu olan robotu yerinde çiviler. Hareketli engel için doğru,
çünkü engel **geçip gidecek**: yoluna 2 saniye girmemek 2 saniyeye mal olur,
girmek bir çarpışmaya.

### Teslim olmadan önce kaçın

Her ileri komutu reddedip sonra durmak, **orijinal hatayı birebir yeniden
üretiyor**: robot duruyor, engel duran robota yürüyor. Ölçüldü — bu kuralın
ilk hali zamanlama taramasındaki çarpışmaların **hiçbirini** düzeltmedi,
ikisini biraz kötüleştirdi.

Bu yüzden hiçbir komut temiz değilse planlayıcı **en çok zaman kazandıran**
komutu alıyor. Geri gitmek pencerededir ve çoğu zaman cevaptır.

> Bu sıralamanın değeri de **varsayılmadı, ölçüldü**: tüm dinamik pencere
> birkaç tehdit geometrisi üzerinde tarandığında, en iyi ve en kötü temas
> süresi arasındaki fark **en fazla 0.1 s** — yani tam olarak bir
> `simulation_dt`. Sıralama gerçek ama tahmin edicinin kendi zaman
> çözünürlüğüyle sınırlı.

### Sadece sert kısıtı zaman-farkında yapmak YETMEDİ

Yalnızca `distanceToCollision` zaman-farkında yapıldığında seçilen komut
statik dünyadakiyle **bit bit aynı** çıktı. Kısıt "durabilir miyim" diye
soruyor, cevap genelde evet, ve robot zaten durmuyor.

Kararı değiştiren şey **clearance puanı**: rollout'u engelin *gideceği* yerden
geçen bir adayın **kötü görünmesi** gerekiyor ki başka bir aday kazansın.

### Tek bir çevrim bunu gösteremez

Dinamik pencere, robotun **bir** kontrol periyodunda ulaşabildiği şeydir:
20 Hz'de 1.0 m/s² limitiyle 0.1 m/s hız aralığı ve ±0.15 rad/s dönüş. Bütün
adaylar neredeyse aynı yere gider, puanlar kılpayı farklıdır, normalizasyondan
sonra sıralama değişmez.

**Kaçınma, çok çevrim boyunca ortaya çıkan bir özelliktir** — bu yüzden onu
kanıtlayan test tek bir `plan()` çağrısı değil, geçiş zamanlamaları üzerinde
taranmış bir **kapalı çevrim** simülasyonu.

## Mini görev

> 1. `planning.yaml`'da `clearance_weight`'i 0.7 → 0.0 yap ve
>    `scripts/evaluate_navigation.py --mode goal --goal goal_room_e` çalıştır.
>    `min_lidar_range_m` ne oluyor? **Neden** hâlâ çarpışmıyor?
> 2. `simulation_time`'ı 2.0 → 0.5 s yap. Robot hangi noktada tıkanıyor?
>    İpucu: 0.7 m/s'de 0.5 s ne kadar yol demek, ve durma mesafesi
>    `v²/(2a)` ne kadar?
> 3. `enforce_stopping_distance: false` yap. Koridor sütununun yanından
>    geçerken ne değişiyor?
