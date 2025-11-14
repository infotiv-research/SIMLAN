#!/bin/bash
HOST_DIR=$(dirname $(dirname "$(pwd)"))
CONTAINER_DIR="/pandoc"
DOCKER_IMAGE="pandoc/extra:latest"
docker build -t "$DOCKER_IMAGE" .

MD_FILES=$(grep -oE '[^:[:space:]]+\.md' "mkdocs.yml" |  tr '\n' ' ')
echo "List of markdown files in mkdocs.yml: $MD_FILES"

docker kill mkdoccontainer
echo "Restarting the docker"
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
        -v "$HOST_DIR/resources/pandoc/mkdocs.yml:/mkdocs.yml" \
        -v "$HOST_DIR:$CONTAINER_DIR" \
        -v $HOME/.ssh:/root/.ssh \
        -v "$(pwd)/output:/output" \
        -p 8000:8000 \
        "$DOCKER_IMAGE" \
        -c "cd $CONTAINER_DIR ; chmod 700 ~/.ssh ; chmod 600 ~/.ssh/config ; chmod 600 ~/.ssh/id_rsa ; chmod 644 ~/.ssh/id_rsa.pub ; $COMMAND"
}


run_command_docker "git config --global --add safe.directory /pandoc ; mkdocs  gh-deploy --config-file /mkdocs.yml --site-dir /output --verbose"

exit

run_command_docker "pandoc --verbose --log=pandoc-log.txt -f markdown-implicit_figures \
        --resource-path=.:resources:resources/pandoc \
        -V geometry:margin=0.5in \
        --table-of-contents \
        --number-sections \
        -V colorlinks -V urlcolor=NavyBlue \
        $MD_FILES  \
        --pdf-engine=xelatex \
        -o $CONTAINER_DIR/resources/pandoc/out.pdf"


run_command_docker "gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$CONTAINER_DIR/resources/pandoc/SIMLAN.pdf  $CONTAINER_DIR/resources/pandoc/out.pdf"

run_command_docker "mkdocs build --config-file /mkdocs.yml --site-dir /output --verbose"


run_command_docker "mkdocs serve --config-file /mkdocs.yml --dev-addr=0.0.0.0:8000"
#run_command_docker "mkdocs gh-deploy  -f /mkdocs.yml  --remote-branch gh-pages"

