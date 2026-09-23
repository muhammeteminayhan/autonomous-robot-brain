# Phase 7 — A*, Dijkstra ve Heuristic Admissibility

## 1. A*, hedefin nerede olduğu söylenmiş Dijkstra'dır

```
g(n)  başlangıçtan n'e bulunan en iyi maliyet
h(n)  n'den hedefe kalan maliyetin TAHMİNİ
f(n)  = g(n) + h(n)
```

**Open set**: keşfedilmiş ama incelenmemiş hücreler, `f`'e göre sıralı.
**Closed set**: incelenmiş hücreler.

```
open'dan en düşük f'li hücreyi çıkar
hedefse bitti
closed'a taşı
her komşu için geçici g hesapla; komşunun mevcut g'sini yeniyorsa kaydet ve open'a it
```

`h = 0` koy → `f = g` → open set başlangıçtan uzaklığa göre sıralanır → bu **tam
olarak Dijkstra**. A* ayrı bir algoritma değil.

## 2. Admissibility — tüm garanti bu

A*, `h` gerçek kalan maliyeti **asla fazla tahmin etmezse** optimal yol döner.
Bu özelliği kaybetmek **hata vermez** — makul görünen, biraz yanlış bir yol
döndürür.

8-komşulu ızgarada çapraz adım `√2` maliyetli ve iki eksende birer hücre
ilerler. Gerçek minimum maliyet **octile** mesafesidir:

```
octile(dx, dy) = (√2 − 1)·min(dx, dy) + max(dx, dy)
```

min(dx,dy) tane çapraz adım at, gerisini düz git.

| Heuristic | Admissible? | Not |
|---|---|---|
| **Octile** | evet, **sıkı** | boş ızgarada gerçek maliyete **eşit** |
| **Euclidean** | evet, gevşek | arada az tahmin eder → fazla genişletme |
| **Manhattan** | **HAYIR** | saf çapraz için `2d` der, gerçeği `1.41d` |
| **Zero** | evet, faydasız | bu Dijkstra |

### Manhattan neden en yaygın planlama hatası?

4-komşulu ızgara için **doğru** heuristic'tir. 8-komşuluda yanlıştır. Ve
çoğu zaman yine de optimali bulur — bu yüzden fark edilmez.

**18 örneklik taramada 10'unda suboptimal, en kötü %8.94 daha uzun.**

Testim önce "her zaman suboptimal" iddia ediyordu ve **başarısız oldu** —
o örnekte Manhattan tesadüfen optimali bulmuştu. İddia "her zaman kötü" değil,
"**garanti yok**"; bir garanti tek karşı örnekle çürütülür.

## 3. Benchmark — ölçülen

Robotun kendi kurduğu haritadan üretilen 12 sorgu, en az 3 m ayrık, her biri
5 koşunun medyanı:

| Algoritma | Heuristic | Ağırlık | Uzunluk [m] | Süre [ms] | Genişletilen | Fazlalık |
|---|---|---|---|---|---|---|
| Dijkstra | yok | 1.0 | 13.211 | 6.764 | 45.836 | %0.00 |
| A* | euclidean | 1.0 | 13.211 | 2.691 | 13.171 | %0.00 |
| **A*** | **octile** | **1.0** | **13.211** | **2.812** | **10.947** | **%0.00** |
| A* | manhattan | 1.0 | 13.174 | 1.658 | 6.573 | **+%1.30** |
| A* | octile | 2.0 | 13.322 | 0.959 | 3.896 | **+%13.99** |
| A* | octile | 5.0 | 13.544 | 0.669 | 2.847 | **+%47.00** |

### Nasıl okunur?

- **A*, Dijkstra'dan 4.2× az düğüm genişletiyor**, aynı yolu buluyor. Bir
  heuristic'in tüm değeri bu.
- **Octile, Euclidean'dan %17 az genişletiyor** (sıkılığın öngördüğü gibi) —
  ama **daha fazla üretiyor** ve duvar-saati süresi başa baş. Genişletme
  sayısı **algoritmik** metrik, süre **mühendislik** metriği; daha sıkı bir
  heuristic kuyruğu daha çok yeniden sıraladığı için ikisi her zaman aynı yöne
  gitmez.
- **Manhattan en hızlı "admissible görünen" seçenek ve admissible değil.**
- **Ağırlıklı A* dürüst bir takas**: ağırlık 5'te Dijkstra'dan 10× hızlı,
  optimalden %47 uzak.

## 4. Costmap — inflation neden?

Engelleri robotun yarıçapı kadar **şişirirsen**, planlayıcı robotu bir **nokta**
sayabilir. Şişirilmiş dünyada nokta planlamak, gerçek dünyada disk planlamakla
**tam olarak eşdeğerdir** ve her hücrede ayak izi kontrol etmekten çok daha ucuz.

Robotun çevrel yarıçapı `√(0.40² + 0.30²)/2 = 0.25 m`; ayarlanan 0.28 m.
Dünyadaki en dar kapı 0.90 m → içinden 0.34 m serbest koridor kalıyor.

### Soft cost neden?

Sadece lethal band olsaydı en kısa yol **her duvara sürtünürdü**. Bu, lokal
planlayıcıya manevra alanı bırakmaz ve her kontrol hatasını çarpışmaya çevirir.
Üstel azalan bir maliyet yolu koridorun ortasında tutar.

### Mesafe dönüşümü

Her engelden çok-kaynaklı **BFS** — harita üzerinde tek geçiş. Naif
uygulama `O(engel × yarıçap²)` splat yapar; 480×360 ızgarada bu fark
milisaniye ile saniye arasındadır.

## 5. Detay gibi görünen, detay olmayan şeyler

**Çapraz adım köşe kesmemeli.** Çaprazda buluşan iki engel, bir **noktanın**
sızabileceği ama bir **diskin** sızamayacağı bir boşluk bırakır. İzin vermek
geometrik olarak en kısa ve fiziksel olarak imkânsız yollar üretir.

**Eşitlikte büyük `g`'yi tercih et.** Aynı `f`'e sahip iki düğüm eşit derecede
umut verici, ama başlangıçtan daha **uzak** olan hedefe daha **yakın**. Onu
önce genişletmek hedefe doğru ilerler; tersi tüm eşit-`f` halkasını açar.

**Bayat kuyruk girdileri.** `std::priority_queue`'da decrease-key yok, bu yüzden
daha iyi bir yolla ulaşılan hücre **tekrar** itilir. Eski kopya yüzeye çıktığında
atılmalı — ve **sadece closed kontrolü yetmez**; `g` kontrolü henüz
genişletilmemiş bir kopyayı yakalar.

**Unknown lethal ama yaymamalı.** Bilinmeyen hücreler geçilmez, ama yanlarındaki
**bilinen serbest alana** soft cost itmemeli; yoksa haritanın sınırı bir duvara
dönüşür ve robot hiç gitmediği hiçbir yere yaklaşmaz.

## Mini görev

> `ros2 launch arb_planning planning.launch.py heuristic:=manhattan` ile
> planlayıcıyı başlat.
>
> 1. `ros2 topic echo /arb/planner_metrics` çıktısında `path_length_m` ve
>    `nodes_expanded` octile'a göre nasıl değişiyor?
> 2. `inflation_radius_m`'i 0.28 → 0.45 yap. Hangi sorgular çözülemez oluyor?
>    **Neden?** (İpucu: en dar kapı 0.90 m.)
> 3. `unknown_is_lethal: false` yap. Robot ne yapmaya kalkışır?
