# demo.ps1 - Demo complete du Yanshee depuis le PC (rien a changer sur le robot)
#
#   powershell -ExecutionPolicy Bypass -File demo.ps1               (marche lente, 5 min)
#   powershell -ExecutionPolicy Bypass -File demo.ps1 -Test         (decide sans bouger le robot)
#   powershell -ExecutionPolicy Bypass -File demo.ps1 -Vitesse normal -Duree 600
#   powershell -ExecutionPolicy Bypass -File demo.ps1 -Carte -Zone 200x150   (reste dans la zone filmee)
#
# 1. verifie le robot, YanAPI.py et la batterie
# 2. lance capteurs_serveur.py (mode distant) dans une autre fenetre
#    (-Carte : et cartographie.py, la webcam au-dessus de la zone)
# 3. ouvre l'interface web
# 4. attend la calibration du capteur de gaz
# 5. fait marcher le robot (Ctrl + C pour l'arreter)

param(
    [string]$Robot = "10.124.7.2",
    [ValidateSet("very slow", "slow", "normal", "fast", "very fast")]
    [string]$Vitesse = "slow",
    [int]$Duree = 300,
    [switch]$Test,
    [switch]$Carte,
    [string]$Zone = "200x150"
)

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot
$serveurLocal = "http://127.0.0.1:8080"

function Etape($texte) { Write-Host ""; Write-Host "==> $texte" -ForegroundColor Cyan }
function Ok($texte) { Write-Host "    OK  $texte" -ForegroundColor Green }
function Stop-Demo($texte) { Write-Host "    ERREUR  $texte" -ForegroundColor Red; exit 1 }
function Lire-Json($url) {
    try { return Invoke-RestMethod -Uri $url -TimeoutSec 3 } catch { return $null }
}

# --- 1. Verifications -------------------------------------------------------
Etape "Verification du robot ($Robot)"
$capteurs = Lire-Json "http://${Robot}:8080/capteurs"
if ($null -eq $capteurs) { Stop-Demo "robot injoignable : meme Wi-Fi ? capteurs_serveur.py lance sur le robot ?" }
if ($capteurs.perime) {
    Stop-Demo ("l'Arduino n'envoie plus rien (derniere mesure il y a {0:N0} s). " -f $capteurs.age +
               "SSH sur le robot : ls /dev/ttyACM* , rebrancher l'Arduino, relancer capteurs_serveur.py")
}
Ok ("capteurs en direct : distance {0} cm, gaz {1}" -f $capteurs.c, $capteurs.gaz)

if (-not $Test) {
    if (-not (Test-Path "YanAPI.py")) { Stop-Demo "YanAPI.py absent : voir README, 'Tester depuis le PC'" }
    Ok "YanAPI.py present"
    $batterie = Lire-Json "http://${Robot}:9090/v1/devices/battery"
    if ($null -eq $batterie) { Stop-Demo "API du robot (port 9090) injoignable" }
    Ok ("batterie {0} %" -f $batterie.data.percent)
    if ($batterie.data.charging -eq 1) {
        Write-Host "    ATTENTION  le robot est en charge : il peut refuser de marcher. Debrancher le chargeur." -ForegroundColor Yellow
    }
    if ($batterie.data.percent -lt 20) {
        Write-Host "    ATTENTION  batterie faible : le robot peut refuser de marcher." -ForegroundColor Yellow
    }
}

# --- 2. Serveur -------------------------------------------------------------
Etape "Lancement du serveur dans une nouvelle fenetre"
if (Lire-Json "$serveurLocal/capteurs") {
    Ok "un serveur tourne deja sur le port 8080 : on le reutilise"
} else {
    $commande = "`$env:SOURCE_HTTP='http://${Robot}:8080'; `$env:ROBOT_IP='$Robot'; " +
                "`$host.UI.RawUI.WindowTitle='Serveur capteurs (fermer = arreter)'; python -W ignore capteurs_serveur.py"
    Start-Process powershell -ArgumentList "-NoExit", "-Command", $commande -WorkingDirectory $PSScriptRoot
    $pret = $false
    for ($i = 0; $i -lt 20; $i++) {
        Start-Sleep -Milliseconds 500
        if (Lire-Json "$serveurLocal/capteurs") { $pret = $true; break }
    }
    if (-not $pret) { Stop-Demo "le serveur ne demarre pas : regarder la fenetre du serveur" }
    Ok "serveur pret"
}

if ($Carte) {
    Etape "Lancement de la webcam (cartographie.py, zone $Zone cm)"
    python -c "import cv2" 2>$null
    if ($LASTEXITCODE -ne 0) { Stop-Demo "OpenCV manquant : python -m pip install opencv-python" }
    if (Lire-Json "http://127.0.0.1:8081/carte") {
        Ok "cartographie.py tourne deja : on le reutilise"
    } else {
        $commande = "`$env:ZONE='$Zone'; `$host.UI.RawUI.WindowTitle='Webcam (fermer = arreter)'; python cartographie.py"
        Start-Process powershell -ArgumentList "-NoExit", "-Command", $commande -WorkingDirectory $PSScriptRoot
    }
    while ($true) {
        $c = Lire-Json "http://127.0.0.1:8081/carte"
        if ($c -and $c.calibre -and $c.robot -and $c.robot.age -lt 1.5) { break }
        $raison = if (-not $c) { "demarrage de la webcam" } elseif (-not $c.calibre) { $c.erreur } else { "robot (marqueur 10) pas vu" }
        Write-Host -NoNewline ("`r    en attente : {0}          " -f $raison)
        Start-Sleep -Seconds 1
    }
    Write-Host ""
    Ok ("zone calibree, robot en x={0} y={1} cm, cap {2}" -f $c.robot.x, $c.robot.y, $c.robot.cap)
    if (-not $c.reference) {
        Write-Host "    ATTENTION  pas de photo du sol vide : obstacles non detectes (bouton 'Photo du sol vide', robot hors zone)" -ForegroundColor Yellow
    }
}

# --- 3. Interface -----------------------------------------------------------
Etape "Ouverture de l'interface : $serveurLocal/"
Start-Process "$serveurLocal/"

# --- 4. Calibration du gaz --------------------------------------------------
Etape "Calibration du capteur de gaz (air propre autour du robot)"
while ($true) {
    $d = Lire-Json "$serveurLocal/capteurs"
    if ($d -and $d.gaz_etat -eq "pret") { break }
    if ($d -and $d.gaz_etat -eq "absent") { Write-Host "    pas de capteur de gaz : on continue sans"; break }
    $reste = if ($d) { $d.gaz_chauffe_s } else { "?" }
    Write-Host -NoNewline ("`r    encore {0} s   " -f $reste)
    Start-Sleep -Seconds 1
}
Write-Host ""
if ($d.gaz_ref) { Ok ("air normal = {0} ; alerte a {1}, danger a {2}" -f $d.gaz_ref, $d.seuils.gaz_alerte, $d.seuils.gaz_danger) }

# --- 5. Marche --------------------------------------------------------------
Etape "Demo"
Write-Host "    Pendant la marche :"
Write-Host "      - main ou obstacle devant l'ultrason -> arret, rotation, il repart"
Write-Host "      - boutons 'Fuite de gaz' / 'Surchauffe' de l'interface -> arret + alerte vocale"
Write-Host "      - Ctrl + C ici -> arret immediat du robot"
if ($Test) { Write-Host "    MODE TEST : le robot ne bougera pas." -ForegroundColor Yellow }
else { Write-Host "    Le robot va MARCHER : sol degage, rester a cote." -ForegroundColor Yellow }
Read-Host "    Entree pour demarrer"

$env:ROBOT_IP = $Robot
$env:URL_CAPTEURS = "$serveurLocal/capteurs"
$env:DUREE_MAX_S = "$Duree"
$arguments = @("-W", "ignore", "marche_obstacle.py", $Vitesse)
if ($Test) { $arguments += "--test" }
if ($Carte) { $arguments += "--carte" }
& python @arguments

Write-Host ""
Write-Host "Fin de la marche. Le serveur et l'interface restent ouverts ; relancer : demo.ps1" -ForegroundColor Cyan
