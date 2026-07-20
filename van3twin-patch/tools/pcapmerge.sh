
DEL_FILTERED=true

if [ -n "$1" ] && [ -n "$2" ]; then
    export SOURCE_PCAP_DIR="$1"
    export BASE_OUTPCAP_DIR="$2"
else
    echo "Usage: $0 <source_pcap_dir> <base_outpcap_dir>"
    exit 1
fi

# ================================ end config params ================================

export RAW_OUTPCAP_DIR=$BASE_OUTPCAP_DIR/raw
mkdir -p "$RAW_OUTPCAP_DIR"
export FILTERED_OUTPCAP_DIR=$BASE_OUTPCAP_DIR/filtered
mkdir -p "$FILTERED_OUTPCAP_DIR"
export MERGED_OUTPCAP_PATH=$BASE_OUTPCAP_DIR/merged.pcap

echo "Moving raw pcap files from $SOURCE_PCAP_DIR to $RAW_OUTPCAP_DIR..."
mv "$SOURCE_PCAP_DIR"/v2v-80211p-student-application-* "$RAW_OUTPCAP_DIR"/

echo "Filtering pcap files and producing TX-only files..."

process_file(){
    file="$1"
    outfile="$FILTERED_OUTPCAP_DIR/$(basename "$file")"

    # Deterministic MAC: extract nodeID from filename, MAC = nodeID + 1
    # ns-3 Mac48Address::Allocate() starts from 00:00:00:00:00:01 sequentially
    nodeid=$(basename "$file" | sed 's/.*application-\([0-9]*\)-0\.pcap/\1/')
    mac_int=$((nodeid + 1))
    EGO_MAC=$(printf "00:00:00:%02x:%02x:%02x" $(( (mac_int >> 16) & 0xFF )) $(( (mac_int >> 8) & 0xFF )) $(( mac_int & 0xFF )))

    tshark -r "$file" -Y "its && wlan.sa == $EGO_MAC" -w "$outfile"
}
export -f process_file

find "$RAW_OUTPCAP_DIR" -maxdepth 1 -type f | parallel --bar -j 16 process_file {}

echo "Done, filtered pcaps are in $FILTERED_OUTPCAP_DIR"
echo "Now i merge all files in one file..."
mergecap -w "$MERGED_OUTPCAP_PATH" "$FILTERED_OUTPCAP_DIR"/*.pcap
echo "Done, merged pcap is in $MERGED_OUTPCAP_PATH"

if [ "$DEL_FILTERED" = true ]; then
    echo "Deleting filtered pcaps..."
    rm "$FILTERED_OUTPCAP_DIR"/*
    rmdir "$FILTERED_OUTPCAP_DIR"
    echo "Done!"
fi
