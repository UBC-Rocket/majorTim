#./fetchBuild.sh :circle-CI-buildNumber

if [ -z "$1" ]; then
	echo "Please specify a CircleCI build number"
	echo "Builds are listed here: https://circleci.com/gh/UBC-Rocket/majorTim"
else
	mkdir -p builds/build-$1
	cd builds/build-$1
	wget https://$1-118362338-gh.circle-artifacts.com/0/root/project/builds/apdet.bin
	wget https://$1-118362338-gh.circle-artifacts.com/0/root/project/builds/sharetest.bin

	if [ -n "$2" ]; then
		#Flash img to a board
		st-flash write $2.bin 0x8000000
	fi
fi
