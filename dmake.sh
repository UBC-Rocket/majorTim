#!/bin/bash
#USAGE:
#Run from the git root
#./dmake.sh telem_mock 	-> builds the telem_mock directory in docker

USER_ID=`id -u`
GROUP_ID=`id -g`
docker run -it --volume `pwd`:/home/rkt/build --user=$USER_ID:$GROUP_ID ubcrocket/majortim-build /bin/sh -c "cd $1 && ./build.sh"
