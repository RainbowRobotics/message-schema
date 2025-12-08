FROM nginx:alpine

RUN apk add --no-cache bash inotify-tools tini

WORKDIR /etc/nginx

COPY watchdog.sh .
COPY nginx.conf .

RUN chmod +x watchdog.sh

EXPOSE 80 443

ENTRYPOINT ["/sbin/tini", "--"]

CMD ["bash", "-c", "nginx -g 'daemon off;' & ./watchdog.sh & wait"]
