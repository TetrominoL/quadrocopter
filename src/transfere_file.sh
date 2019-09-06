#!/bin/sh

IP="10.10.10.1"
PORT="12345"

if [ -z "$1" ] || [ "$1" = "-h" ]; then
    echo "Usage: $(basename $0) <File to Transfere> <opt. IP> <opt. PORT>"
    echo "\tdefault is PORT=$PORT and IP=$IP"
    exit 0;
fi

if [ -n "$2" ]; then
    IP=$2
fi

if [ -n "$3" ]; then
    PORT=$3
fi

nc $IP $PORT < $1