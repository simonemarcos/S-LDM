FROM debian:bookworm-slim

ENV USERNAME=activemquser
ENV ACTIVEMQ_VERSION=5.19.8
ENV USERHOME=/home/${USERNAME}
ENV ACTIVEMQ_HOME=${USERHOME}/.activemq
ENV SDKMAN_DIR=${USERHOME}/.sdkman

SHELL ["/bin/bash", "-c"]

# install dependencies as root
RUN apt-get update && \
    apt-get install -y wget curl zip unzip bash ca-certificates gnupg && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# create user and home directory
RUN useradd -m -s /bin/bash ${USERNAME}
USER ${USERNAME}
WORKDIR ${USERHOME}

# install SDKMAN and Java 21 temurin
RUN (curl -s "https://get.sdkman.io" | bash) && \
    source $SDKMAN_DIR/bin/sdkman-init.sh && \
    sdk install java 21-tem

ENV JAVA_HOME=${SDKMAN_DIR}/candidates/java/current
ENV PATH=${JAVA_HOME}/bin:$PATH

# install ActiveMQ
RUN wget -q https://dlcdn.apache.org/activemq/${ACTIVEMQ_VERSION}/apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz && \
    wget -q https://dlcdn.apache.org/activemq/${ACTIVEMQ_VERSION}/apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz.sha512 && \
    (sha512sum -c apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz.sha512) && \
    echo "Successful CheckSum sha512" && \
    rm apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz.sha512 && \
    tar -xzf apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz -C ./ && \
    rm apache-activemq-${ACTIVEMQ_VERSION}-bin.tar.gz && \
    mv ./apache-activemq-${ACTIVEMQ_VERSION} ${ACTIVEMQ_HOME}


# set access to 0.0.0.0
RUN sed -i 's/<property name="host" value="127.0.0.1"\/>/<property name="host" value="0.0.0.0"\/>/' ${ACTIVEMQ_HOME}/conf/jetty.xml

# Remove 'upgrade-insecure-requests;' from Content-Security-Policy headers.
# This CSP directive tells browsers to auto-upgrade HTTP URLs to HTTPS,
# which breaks all CSS/links/resources when serving over plain HTTP.
RUN sed -i 's/upgrade-insecure-requests; //g' ${ACTIVEMQ_HOME}/conf/jetty.xml

EXPOSE 8161 61616 5672 61613 5867

CMD ["/home/activemquser/.activemq/bin/activemq", "console"]