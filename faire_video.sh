#!/bin/bash
# =====================================================================
#  faire_video.sh : montage de la vidéo de présentation Robot Parking
#  Produit ~/Videos/robot_parking_demo.mp4 (1280x720, 30 i/s, ~80 s)
#  Utilisation :  bash faire_video.sh
# =====================================================================
set -e
V=~/Videos
I=~/parking_ws/src/robot_parking/docs/images
T=$(mktemp -d)                      # dossier de travail temporaire
SORTIE=$V/robot_parking_demo.mp4
FOND=0x1e2a38                       # bleu nuit (fond des écrans titre)
POLICE=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
POLICE2=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
ENC="-c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r 30 -an"
CADRE="scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2:color=$FOND"
n=0

# Bandeau de légende en bas de l'image (le texte est lu dans un fichier : accents et apostrophes OK)
bandeau() {
  echo "drawbox=x=0:y=ih-96:w=iw:h=96:color=black@0.6:t=fill,drawtext=fontfile=$POLICE:textfile=$1:fontcolor=white:fontsize=32:x=40:y=h-64"
}
fondus() {   # fondu d'entrée et de sortie de 0,4 s
  echo "fade=t=in:st=0:d=0.4,fade=t=out:st=$(awk "BEGIN{print $1-0.4}"):d=0.4"
}

# --- Écran titre / écran de fin : 2 lignes de texte sur fond uni
ecran() {   # ecran DUREE "grande ligne" ["petite ligne"]
  n=$((n+1)); printf '%s' "$2" > $T/l1_$n.txt
  if [ -n "$3" ]; then
    printf '%s' "$3" > $T/l2_$n.txt
    textes="drawtext=fontfile=$POLICE:textfile=$T/l1_$n.txt:fontcolor=white:fontsize=64:x=(w-tw)/2:y=(h-th)/2-40,drawtext=fontfile=$POLICE2:textfile=$T/l2_$n.txt:fontcolor=0xF2A65A:fontsize=32:x=(w-tw)/2:y=(h-th)/2+45"
  else
    textes="drawtext=fontfile=$POLICE:textfile=$T/l1_$n.txt:fontcolor=white:fontsize=80:x=(w-tw)/2:y=(h-th)/2"
  fi
  ffmpeg -y -loglevel error -f lavfi -i color=c=$FOND:s=1280x720:d=$1:r=30 -vf \
    "$textes,$(fondus $1)" $ENC -t $1 $T/s$(printf %02d $n).mp4
  echo "  séquence $n : écran « $2 »"
}

# --- Image fixe avec bandeau
image() {   # image FICHIER DUREE "légende" [recadrage L:H:X:Y]
  n=$((n+1)); printf '%s' "$3" > $T/b_$n.txt
  recadre=""; [ -n "$4" ] && recadre="crop=$4,"
  ffmpeg -y -loglevel error -loop 1 -t $2 -i "$1" -vf "$recadre$CADRE,$(bandeau $T/b_$n.txt),$(fondus $2)" \
    $ENC -t $2 $T/s$(printf %02d $n).mp4
  echo "  séquence $n : image $(basename "$1")"
}

# --- Vidéo accélérée pour durer exactement DUREE secondes, avec bandeau
video() {   # video FICHIER DEBUT DUREE "légende"
  n=$((n+1)); printf '%s' "$4" > $T/b_$n.txt
  total=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$1")
  accel=$(awk "BEGIN{print ($total-$2)/$3}")
  ffmpeg -y -loglevel error -ss $2 -i "$1" -vf \
    "setpts=PTS/$accel,fps=30,$CADRE,$(bandeau $T/b_$n.txt),$(fondus $3)" \
    $ENC -t $3 $T/s$(printf %02d $n).mp4
  echo "  séquence $n : $(basename "$1") (accéléré x$(printf %.1f $accel))"
}

echo "Montage en cours (environ 2 minutes)..."
ecran 3   "Robot Parking"

ecran 2.5 "Partie 1" "Navigation autonome"
image "$I/etape4_astar.png"            3.5 "Navigation · Carte du parking et chemin A*"
video "$V/etape5.webm"          1 7   "Navigation · Suivi autonome du chemin"
video "$V/etape6.webm"          1 7   "Navigation · Arrêt de sécurité devant un obstacle"
video "$V/D2_obstacle.webm"     1 8   "Navigation · Re-planification autour de l'obstacle"
video "$V/D2b_attente_5s.webm"  1 7   "Navigation · Priorité piéton : il attend, puis repart"
video "$V/D3_dwa_pieton.webm"   1 7   "Navigation · DWA : il anticipe le piéton"
video "$V/D4_ppo_pieton.webm"   1 6   "Navigation · PPO (renforcement) + bouclier"
image "$I/D5_comparaison.png"          6 "Navigation · Comparaison des 3 conducteurs"

ecran 2.5 "Partie 2" "Vision"
video "$V/vision_B.webm"        1 7   "Vision · YOLOv8 + lecture de la plaque (OpenCV)"
image "$I/vision_B_camera_plaque.png"  3.5 "Vision · La caméra lit la plaque tunisienne" "1100:894:738:103"
video "$V/vision_tournee.webm"  2 9   "Vision · Retrouver une voiture par sa plaque"

ecran 4.5 "ROS 2 · Gazebo · A* · DWA · PPO · YOLOv8" "github.com/esmat1103/robot_parking"

# --- Assemblage final (toutes les séquences ont le même format -> copie sans réencodage)
ls $T/s*.mp4 | sort | sed "s/^/file '/; s/$/'/" > $T/liste.txt
ffmpeg -y -loglevel error -f concat -safe 0 -i $T/liste.txt -c copy -movflags +faststart "$SORTIE"
rm -rf $T
echo "Vidéo prête : $SORTIE ($(ffprobe -v error -show_entries format=duration -of csv=p=0 "$SORTIE" | cut -d. -f1) s)"
