# UAV Physics Simulator 🚁

Multirotor UAV aerodynamics & endurance simulator — runs as a PWA on Android/Desktop.

## ⚡ Deploy to GitHub Pages (no coding needed)

### Step 1 — GitHub hesabı aç
1. **github.com** → **Sign up** (pulsuz)
2. Email ilə qeydiyyat keç

### Step 2 — Yeni repo yarat
1. Sağ üstdə **+** → **New repository**
2. **Repository name**: `uav-simulator`
3. **Public** seç (mütləq — GitHub Pages üçün lazımdır)
4. **Create repository** düyməsinə bas

### Step 3 — Faylları yüklə
1. Repo səhifəsində **uploading an existing file** linkinə bas
2. Bu qovluqdakı **bütün faylları** sürüklə-burax:
   ```
   uav-simulator/
   ├── index.html
   ├── package.json
   ├── vite.config.js
   ├── src/
   │   ├── main.jsx
   │   └── App.jsx
   ├── public/
   │   ├── manifest.json
   │   ├── icon.svg
   │   └── icon-192.png  (lazım deyil, amma varsa yaxşıdır)
   └── .github/
       └── workflows/
           └── deploy.yml
   ```
3. **Commit changes** düyməsinə bas

### Step 4 — GitHub Pages aktiv et
1. Repo → **Settings** → sol menyuda **Pages**
2. **Source**: `GitHub Actions` seç
3. Yadda saxla

### Step 5 — Deploy gözlə
1. Repo → **Actions** tab
2. Sarı dairə görəcəksən (işləyir) → yaşıl ✓ (hazır!)
3. ~2-3 dəqiqə gözlə

### Step 6 — Linkini tap
1. **Settings** → **Pages**
2. `https://SENİN-ADIN.github.io/uav-simulator/` linki görünəcək
3. Bu linki dostlarınla paylaş! 🎉

---

## 📱 Android-də App kimi yükle

1. Chrome-da linki aç
2. Ünvan çubuğunun yanında **"Quraşdır"** ikonu görünəcək
3. Və ya sağ üst **⋮** menyusu → **Ana ekrana əlavə et**
4. İndi app kimi açılır — internet lazım deyil (offline işləyir)!

---

## 🔄 Kodu yeniləmək istəsən

1. GitHub-da `src/App.jsx` faylını tap
2. ✏️ (edit) düyməsinə bas
3. Yeni kodu yapışdır
4. **Commit** et → avtomatik deploy olur

---

## ⚙️ Lokal test (isteğe bağlı)

```bash
npm install
npm run dev
# http://localhost:5173 açılır
```
