#!/bin/bash

# Function to rename files recursively
rename_files() {
    local mode=$1
    for file in "$2"/*; do
        if [ -d "$file" ]; then
            # If it's a directory, recurse into it
            rename_files "$mode" "$file"
        elif [ -f "$file" ]; then
            # If it's a file, process it
            directory=$(dirname "$file")
            filename=$(basename "$file")
            extension="${filename##*.}"
            filename="${filename%.*}"
            
            # Generate new name
            new_name="${directory}/${filename}OLD.${extension}"
            
            if [ "$mode" == "preview" ]; then
                echo "Would rename: $file -> $new_name"
            elif [ "$mode" == "rename" ]; then
                mv "$file" "$new_name"
                echo "Renamed: $file -> $new_name"
            fi
        fi
    done
}

# Check for command line argument
if [ "$1" == "--preview" ]; then
    echo "Preview mode: No files will be renamed"
    rename_files "preview" "."
elif [ "$1" == "--rename" ]; then
    echo "Renaming files..."
    rename_files "rename" "."
else
    echo "Usage: $0 [--preview|--rename]"
    echo "  --preview: Show what files would be renamed without making changes"
    echo "  --rename: Actually rename the files"
    exit 1
fi

echo "Operation complete. Please review the changes."
