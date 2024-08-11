#!/bin/bash

# Function to rename folders recursively
rename_folders() {
    local mode=$1
    local current_dir="$2"
    
    # First, process subdirectories (depth-first)
    for item in "$current_dir"/*; do
        if [ -d "$item" ]; then
            rename_folders "$mode" "$item"
        fi
    done
    
    # Then, rename the current directory if it's not the root
    if [ "$current_dir" != "." ]; then
        directory=$(dirname "$current_dir")
        foldername=$(basename "$current_dir")
        
        # Generate new name
        new_name="${directory}/${foldername}OLD"
        
        if [ "$mode" == "preview" ]; then
            echo "Would rename: $current_dir -> $new_name"
        elif [ "$mode" == "rename" ]; then
            mv "$current_dir" "$new_name"
            echo "Renamed: $current_dir -> $new_name"
        fi
    fi
}

# Check for command line argument
if [ "$1" == "--preview" ]; then
    echo "Preview mode: No folders will be renamed"
    rename_folders "preview" "."
elif [ "$1" == "--rename" ]; then
    echo "Renaming folders..."
    rename_folders "rename" "."
else
    echo "Usage: $0 [--preview|--rename]"
    echo "  --preview: Show what folders would be renamed without making changes"
    echo "  --rename: Actually rename the folders"
    exit 1
fi

echo "Operation complete. Please review the changes."
