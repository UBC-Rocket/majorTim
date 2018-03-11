#!/bin/bash

if [ $# -eq 0 ]; then #make sure we get some params into the script
	echo -e "USAGE:\n
	Note: Run from the git root\n
	./dmake.sh telem_mock		-> builds the telem_mock directory in docker
	./dmake.sh clean			-> cleans up the auto generated object files"
	exit 1
fi

FilePath=(
	"apdet-hanna"		#FilePath[0]
	"sharetest"			#FilePath[1]
	"telem"				#FilePath[2]
)

if [ $1 = clean ]; then

	for Path in "${FilePath[@]}"
	do
		cd $Path
		./build.sh clean
		cd ..
	done
	exit 0
fi

USER_ID=`id -u`
GROUP_ID=`id -g`
docker run -it --volume "`pwd`":/home/rkt/build --user=$USER_ID:$GROUP_ID ubcrocket/majortim-build /bin/sh -c "cd $1 && ./build.sh"
