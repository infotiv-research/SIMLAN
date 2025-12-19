#!/bin/bash
HOST_DIR=$(dirname $(dirname "$(pwd)"))
CONTAINER_DIR="/docvolume"
DOCKER_IMAGE="pandoc/extra:latest"
docker build -t "$DOCKER_IMAGE" .
MD_FILES=$(grep -oE '[^:[:space:]]+\.md' "mkdocs.yml" |  tr '\n' ' ')
echo "List of markdown files in mkdocs.yml: $MD_FILES"

## CLEAN UP
docker kill mkdoccontainer
echo "Restarting the docker"
rm -f *.pdf
rm -rf output
cd ../../ ; ./control.sh clean
sleep 3

run_command_docker() {
    local COMMAND="$1"
    if [ -z "$COMMAND" ]; then
        echo "Error: No command provided to run_command_docker." >&2
        return 1
    fi

    echo "Running command inside Docker: $COMMAND"
    # Execute the docker run command
    docker run --rm \
        --name mkdoccontainer \
        --entrypoint /bin/sh \
        -v "$HOST_DIR/resources/build-documentation/mkdocs.yml:/mkdocs.yml" \
        -v "$HOST_DIR:$CONTAINER_DIR" \
        -v "$HOME/.ssh:/tmp/ssh_mount:ro" \
        -v "$(pwd)/output:/output" \
        -p 8000:8000 \
        "$DOCKER_IMAGE" \
        -c "
        mkdir -p /root/.ssh && \
        cp -r /tmp/ssh_mount/* /root/.ssh/ && \
        chmod 700 /root/.ssh && \
        chmod 600 /root/.ssh/config 2>/dev/null || true && \
        chmod 600 /root/.ssh/id_rsa 2>/dev/null || true && \
        chmod 644 /root/.ssh/id_rsa.pub 2>/dev/null || true && \
        chown -R root:root /root/.ssh && \
        cd $CONTAINER_DIR && \
        $COMMAND
        "
}

run_command_docker "pandoc --verbose --log=pandoc-log.txt -f markdown-implicit_figures \
        --resource-path=.:resources:resources/build-documentation \
        -V geometry:margin=0.5in \
        --table-of-contents \
        --number-sections \
        -V colorlinks -V urlcolor=NavyBlue \
        $MD_FILES  \
        --pdf-engine=xelatex \
        -o $CONTAINER_DIR/resources/build-documentation/tmp.pdf"

run_command_docker "gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$CONTAINER_DIR/SIMLAN.pdf  $CONTAINER_DIR/resources/build-documentation/tmp.pdf"

run_command_docker "mkdocs build --config-file /mkdocs.yml --site-dir /output --verbose"
run_command_docker "mkdocs serve --config-file /mkdocs.yml --dev-addr=0.0.0.0:8000"

# THIS HAS TO BE EXECUTED IN THE GITHUB REPO (NOT INTERNAL GITLAB)
# run_command_docker "git config --global --add safe.directory /pandoc ; mkdocs  gh-deploy --config-file /mkdocs.yml --site-dir /output --verbose"


