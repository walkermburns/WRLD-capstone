REMOTE_USER="walker"
REMOTE_IP="raspberrypi"
REMOTE_DEST="~/"
LOCAL_SOURCE="./buoy"


echo "Copying files from $LOCAL_SOURCE to ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DEST}"
rsync -avz --delete --ignore-times $LOCAL_SOURCE ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_DEST}
echo "Finished copying files."