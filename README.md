# voice_system
The XunFei voice using in ROS.

We should install this application in order to play the .wav file.

```
sudo apt-get install sox
sudo apt-get install libjsoncpp1 libjsoncpp-dev
sudo apt-get install libcurl3 libcurl4-openssl-dev
```
Importantly,we must copy the libmsc.so file to /usr/lib:
```
sudo cp libmsc.so /usr/lib
```

Then we should register an tuling account in www.tuling123.com

To launch the file,we start three nodes by using this command:
```
roslaunch voice_system voice.launch
```

Next,we should issue an order by:
```
rostopic pub -1 /voice/xf_asr_topic std_msgs/Int32 1
```
Finally,we can say something to the computer,and it will answer your question.
