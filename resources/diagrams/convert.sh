rm *.png

########################################################################
## install drawio package first on the HOST machine, if you have snap:##
## snap install drawio						      ##
## cd into this directory and run this script			      ##
########################################################################

rm README.md
for file in *.drawio; do
  [ -f "$file" ] || continue
  drawio --export --format png --scale 3 --transparent "$file" --output "$file.png"
  echo "###" $file  >> README.md
  echo "![\"$file.png\"](<$file.png>)" >> README.md
done
