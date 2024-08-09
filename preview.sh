kicad-cli sch export svg drone.kicad_sch -o output_image
#!/bin/bash

# Clean SVG files
for file in output_image/*.svg; do
    sed -i 's/[\x01-\x1F\x7F]//g' "$file"
done

# Convert to PNG and remove SVG
for file in output_image/*.svg; do
    inkscape "$file" --export-type=png --export-dpi=300 --export-filename="${file%.svg}.png" 2>/dev/null && rm "$file"
done