'''
    @file: detectnet.py
    @brief: object detection using mobilenetv2 on ndivia jetson nano 2gb 
    @command: python3 detectnet.py
    @author: khang nguyen - rvl
'''
#! usr/bin/python3

# Import neccessary libraries
import sys
import argparse
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, logUsage

# Parse the command line
parser = argparse.ArgumentParser(description="Object Detection using MobileNetv2.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
#parser.add_argument("--network", type=str, default="resnet-18", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 
is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

# Customize video sources and outputs
input = videoSource("/dev/video2", ["--input-width=480", "--input-height=320"])
output = videoOutput(args.output_URI, argv=sys.argv+is_headless)
	
# Load detection network
net = detectNet(args.network, sys.argv, args.threshold)

while True:
	# Capture single image and detect objects on the image
	img = input.Capture()
	detections = net.Detect(img, overlay=args.overlay)

	# Stream the updated detected image and get frame rate
	output.Render(img)
	output.SetStatus("Detection at {:.0f} FPS".format(net.GetNetworkFPS()))

	# Break the stream
	if not input.IsStreaming() or not output.IsStreaming():
		break
