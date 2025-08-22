FROM espressif/idf:release-v5.4.2
RUN apt update
RUN apt-get install -y curl
RUN curl -fsSL https://deb.nodesource.com/setup_18.x -o nodesource_setup.sh
RUN bash nodesource_setup.sh
RUN apt -y install nodejs