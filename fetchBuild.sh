#./fetchBuild.sh :circle-CI-buildNumber

if [ -z "$var" ]; then
	echo "Please specify a circleCI build number"
	echo "Builds are listed here: https://circleci.com/gh/UBC-Rocket/majorTim"
else
	mkdir builds
	cd builds
	mkdir build-$1
	cd build-$1
	wget https://$1-118362338-gh.circle-artifacts.com/0/tmp/apdet_hanna.bin
fi
