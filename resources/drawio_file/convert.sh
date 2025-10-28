rm *.png

#########################################################################
####### install drawio package first on the HOST machine
####### cd into this directory and run this script
#########################################################################

for file in *.drawio; do
  [ -f "$file" ] || continue
  drawio --export --format png --scale 3 --transparent "$file" --output "$file.png"
done