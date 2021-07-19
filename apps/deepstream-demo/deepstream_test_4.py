#!/usr/bin/env python3

################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
################################################################################

import sys
sys.path.append('../')

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import GLib, GObject, Gst, GstRtspServer
import platform
from optparse import OptionParser
from common.is_aarch_64 import is_aarch64
from common.bus_call import bus_call
from common.utils import long_to_int
import pyds

MAX_DISPLAY_LEN=64
MAX_TIME_STAMP_LEN=32
PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3
MUXER_OUTPUT_WIDTH=1920
MUXER_OUTPUT_HEIGHT=1080
MUXER_BATCH_TIMEOUT_USEC=4000000
input_file = None
schema_type = 0
frame_number = 0
proto_lib = None
conn_str="localhost;2181;testTopic"
cfg_file = None
topic = None
no_display = False

PGIE_CONFIG_FILE="dstest4_pgie_config.txt"
MSCONV_CONFIG_FILE="dstest4_msgconv_config.txt"


pgie_classes_str=["Vehicle", "TwoWheeler", "Person","Roadsign"]

# Callback function for deep-copying an NvDsEventMsgMeta struct
def meta_copy_func(data,user_data):
    # Cast data to pyds.NvDsUserMeta
    user_meta=pyds.NvDsUserMeta.cast(data)
    src_meta_data=user_meta.user_meta_data
    # Cast src_meta_data to pyds.NvDsEventMsgMeta
    srcmeta=pyds.NvDsEventMsgMeta.cast(src_meta_data)
    # Duplicate the memory contents of srcmeta to dstmeta
    # First use pyds.get_ptr() to get the C address of srcmeta, then
    # use pyds.memdup() to allocate dstmeta and copy srcmeta into it.
    # pyds.memdup returns C address of the allocated duplicate.
    dstmeta_ptr=pyds.memdup(pyds.get_ptr(srcmeta), sys.getsizeof(pyds.NvDsEventMsgMeta))
    # Cast the duplicated memory to pyds.NvDsEventMsgMeta
    dstmeta=pyds.NvDsEventMsgMeta.cast(dstmeta_ptr)

    # Duplicate contents of ts field. Note that reading srcmeat.ts
    # returns its C address. This allows to memory operations to be
    # performed on it.
    dstmeta.ts=pyds.memdup(srcmeta.ts, MAX_TIME_STAMP_LEN+1)

    # Copy the sensorStr. This field is a string property.
    # The getter (read) returns its C address. The setter (write)
    # takes string as input, allocates a string buffer and copies
    # the input string into it.
    # pyds.get_string() takes C address of a string and returns
    # the reference to a string object and the assignment inside the binder copies content.
    dstmeta.sensorStr=pyds.get_string(srcmeta.sensorStr)

    if(srcmeta.objSignature.size>0):
        dstmeta.objSignature.signature=pyds.memdup(srcmeta.objSignature.signature,srcMeta.objSignature.size)
        dstmeta.objSignature.size = srcmeta.objSignature.size;

    if(srcmeta.extMsgSize>0):
        if(srcmeta.objType==pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE):
            srcobj = pyds.NvDsVehicleObject.cast(srcmeta.extMsg);
            obj = pyds.alloc_nvds_vehicle_object();
            obj.type=pyds.get_string(srcobj.type)
            obj.make=pyds.get_string(srcobj.make)
            obj.model=pyds.get_string(srcobj.model)
            obj.color=pyds.get_string(srcobj.color)
            obj.license = pyds.get_string(srcobj.license)
            obj.region = pyds.get_string(srcobj.region)
            dstmeta.extMsg = obj;
            dstmeta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject)
        if(srcmeta.objType==pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON):
            srcobj = pyds.NvDsPersonObject.cast(srcmeta.extMsg);
            obj = pyds.alloc_nvds_person_object()
            obj.age = srcobj.age
            obj.gender = pyds.get_string(srcobj.gender);
            obj.cap = pyds.get_string(srcobj.cap)
            obj.hair = pyds.get_string(srcobj.hair)
            obj.apparel = pyds.get_string(srcobj.apparel);
            dstmeta.extMsg = obj;
            dstmeta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject);

    return dstmeta

# Callback function for freeing an NvDsEventMsgMeta instance
def meta_free_func(data,user_data):
    user_meta=pyds.NvDsUserMeta.cast(data)
    srcmeta=pyds.NvDsEventMsgMeta.cast(user_meta.user_meta_data)

    # pyds.free_buffer takes C address of a buffer and frees the memory
    # It's a NOP if the address is NULL
    pyds.free_buffer(srcmeta.ts)
    pyds.free_buffer(srcmeta.sensorStr)

    if(srcmeta.objSignature.size > 0):
        pyds.free_buffer(srcmeta.objSignature.signature);
        srcmeta.objSignature.size = 0

    if(srcmeta.extMsgSize > 0):
        if(srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE):
            obj =pyds.NvDsVehicleObject.cast(srcmeta.extMsg)
            pyds.free_buffer(obj.type);
            pyds.free_buffer(obj.color);
            pyds.free_buffer(obj.make);
            pyds.free_buffer(obj.model);
            pyds.free_buffer(obj.license);
            pyds.free_buffer(obj.region);
        if(srcmeta.objType == pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON):
            obj = pyds.NvDsPersonObject.cast(srcmeta.extMsg);
            pyds.free_buffer(obj.gender);
            pyds.free_buffer(obj.cap);
            pyds.free_buffer(obj.hair);
            pyds.free_buffer(obj.apparel);
        pyds.free_gbuffer(srcmeta.extMsg);
        srcmeta.extMsgSize = 0;

def generate_vehicle_meta(data):
    obj = pyds.NvDsVehicleObject.cast(data);
    obj.type ="sedan"
    obj.color="blue"
    obj.make ="Bugatti"
    obj.model = "M"
    obj.license ="XX1234"
    obj.region ="CA"
    return obj

def generate_person_meta(data):
    obj = pyds.NvDsPersonObject.cast(data)
    obj.age = 45
    obj.cap = "none"
    obj.hair = "black"
    obj.gender = "male"
    obj.apparel= "formal"
    return obj

def generate_event_msg_meta(data, class_id):
    meta =pyds.NvDsEventMsgMeta.cast(data)
    meta.sensorId = 0
    meta.placeId = 0
    meta.moduleId = 0
    meta.sensorStr = "sensor-0"
    meta.ts = pyds.alloc_buffer(MAX_TIME_STAMP_LEN + 1)
    pyds.generate_ts_rfc3339(meta.ts, MAX_TIME_STAMP_LEN)

    # This demonstrates how to attach custom objects.
    # Any custom object as per requirement can be generated and attached
    # like NvDsVehicleObject / NvDsPersonObject. Then that object should
    # be handled in payload generator library (nvmsgconv.cpp) accordingly.
    if(class_id==PGIE_CLASS_ID_VEHICLE):
        meta.type = pyds.NvDsEventType.NVDS_EVENT_MOVING
        meta.objType = pyds.NvDsObjectType.NVDS_OBJECT_TYPE_VEHICLE
        meta.objClassId = PGIE_CLASS_ID_VEHICLE
        obj = pyds.alloc_nvds_vehicle_object()
        obj = generate_vehicle_meta(obj)
        meta.extMsg = obj
        meta.extMsgSize = sys.getsizeof(pyds.NvDsVehicleObject);
    if(class_id == PGIE_CLASS_ID_PERSON):
        meta.type =pyds.NvDsEventType.NVDS_EVENT_ENTRY
        meta.objType = pyds.NvDsObjectType.NVDS_OBJECT_TYPE_PERSON;
        meta.objClassId = PGIE_CLASS_ID_PERSON
        obj = pyds.alloc_nvds_person_object()
        obj=generate_person_meta(obj)
        meta.extMsg = obj
        meta.extMsgSize = sys.getsizeof(pyds.NvDsPersonObject)
    return meta

# osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
# and update params for drawing rectangle, object information etc.
# IMPORTANT NOTE:
# a) probe() callbacks are synchronous and thus holds the buffer
#    (info.get_buffer()) from traversing the pipeline until user return.
# b) loops inside probe() callback could be costly in python.
#    So users shall optimize according to their use-case.
def osd_sink_pad_buffer_probe(pad,info,u_data):
    frame_number=0
    #Intiallizing object counter with 0.
    obj_counter = {
        PGIE_CLASS_ID_VEHICLE:0,
        PGIE_CLASS_ID_PERSON:0,
        PGIE_CLASS_ID_BICYCLE:0,
        PGIE_CLASS_ID_ROADSIGN:0
    }
    num_rects=0
    is_first_object=True
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    if not batch_meta:
        return Gst.PadProbeReturn.OK
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            continue
        is_first_object = True;

        '''
        print("Frame Number is ", frame_meta.frame_num)
        print("Source id is ", frame_meta.source_id)
        print("Batch id is ", frame_meta.batch_id)
        print("Source Frame Width ", frame_meta.source_frame_width)
        print("Source Frame Height ", frame_meta.source_frame_height)
        print("Num object meta ", frame_meta.num_obj_meta)
        '''
        frame_number=frame_meta.frame_num
        num_rects = frame_meta.num_obj_meta
        l_obj=frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                continue

            # Update the object text display
            txt_params=obj_meta.text_params

            # Set display_text. Any existing display_text string will be
            # freed by the bindings module.
            txt_params.display_text = pgie_classes_str[obj_meta.class_id]

            obj_counter[obj_meta.class_id] += 1

            # Font , font-color and font-size
            txt_params.font_params.font_name = "Serif"
            txt_params.font_params.font_size = 10
            # set(red, green, blue, alpha); set to White
            txt_params.font_params.font_color.set(1.0, 1.0, 1.0, 1.0);

            # Text background color
            txt_params.set_bg_clr = 1
            # set(red, green, blue, alpha); set to Black
            txt_params.text_bg_clr.set(0.0, 0.0, 0.0, 1.0);

            # Ideally NVDS_EVENT_MSG_META should be attached to buffer by the
            # component implementing detection / recognition logic.
            # Here it demonstrates how to use / attach that meta data.
            if(is_first_object and not (frame_number%30)):
            # if(is_first_object and not (frame_number%1)):
                # Frequency of messages to be send will be based on use case.
                # Here message is being sent for first object every 30 frames.

                # Allocating an NvDsEventMsgMeta instance and getting reference
                # to it. The underlying memory is not manged by Python so that
                # downstream plugins can access it. Otherwise the garbage collector
                # will free it when this probe exits.
                msg_meta=pyds.alloc_nvds_event_msg_meta()
                msg_meta.bbox.top =  obj_meta.rect_params.top
                msg_meta.bbox.left =  obj_meta.rect_params.left
                msg_meta.bbox.width = obj_meta.rect_params.width
                msg_meta.bbox.height = obj_meta.rect_params.height
                msg_meta.frameId = frame_number
                msg_meta.trackingId = long_to_int(obj_meta.object_id)
                msg_meta.confidence = obj_meta.confidence
                msg_meta = generate_event_msg_meta(msg_meta, obj_meta.class_id)
                user_event_meta = pyds.nvds_acquire_user_meta_from_pool(batch_meta)
                if(user_event_meta):
                    user_event_meta.user_meta_data = msg_meta;
                    user_event_meta.base_meta.meta_type = pyds.NvDsMetaType.NVDS_EVENT_MSG_META
                    # Setting callbacks in the event msg meta. The bindings layer
                    # will wrap these callables in C functions. Currently only one
                    # set of callbacks is supported.
                    pyds.set_user_copyfunc(user_event_meta, meta_copy_func)
                    pyds.set_user_releasefunc(user_event_meta, meta_free_func)
                    pyds.nvds_add_user_meta_to_frame(frame_meta, user_event_meta)
                else:
                    print("Error in attaching event meta to buffer\n")

                is_first_object = False
            try:
                l_obj=l_obj.next
            except StopIteration:
                break
        try:
            l_frame=l_frame.next
        except StopIteration:
            break

    print("Frame Number =",frame_number,"Vehicle Count =",obj_counter[PGIE_CLASS_ID_VEHICLE],"Person Count =",obj_counter[PGIE_CLASS_ID_PERSON])
    return Gst.PadProbeReturn.OK


def main(args):
    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")

    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")

    # Source element for reading from the file
    print("Creating Source \n ")
    source = Gst.ElementFactory.make("filesrc", "file-source")
    if not source:
        sys.stderr.write(" Unable to create Source \n")

    # Since the data format in the input file is elementary h264 stream,
    # we need a h264parser
    print("Creating H264Parser \n")
    h264parser = Gst.ElementFactory.make("h264parse", "h264-parser")
    if not h264parser:
        sys.stderr.write(" Unable to create h264 parser \n")

    # Use nvdec_h264 for hardware accelerated decode on GPU
    print("Creating Decoder \n")
    decoder = Gst.ElementFactory.make("nvv4l2decoder", "nvv4l2-decoder")
    if not decoder:
        sys.stderr.write(" Unable to create Nvv4l2 Decoder \n")

    # Create nvstreammux instance to form batches from one or more sources.
    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")

    # Use nvinfer to run inferencing on decoder's output,
    # behaviour of inferencing is set through config file
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")

    # Use convertor to convert from NV12 to RGBA as required by nvosd
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")

    # Create OSD to draw on the converted RGBA buffer
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")

    msgconv=Gst.ElementFactory.make("nvmsgconv", "nvmsg-converter")
    if not msgconv:
        sys.stderr.write(" Unable to create msgconv \n")

    msgbroker=Gst.ElementFactory.make("nvmsgbroker", "nvmsg-broker")
    if not msgbroker:
        sys.stderr.write(" Unable to create msgbroker \n")

    tee=Gst.ElementFactory.make("tee", "nvsink-tee")
    if not tee:
        sys.stderr.write(" Unable to create tee \n")

    queue1=Gst.ElementFactory.make("queue", "nvtee-que1")
    if not queue1:
        sys.stderr.write(" Unable to create queue1 \n")

    queue2=Gst.ElementFactory.make("queue", "nvtee-que2")
    if not queue2:
        sys.stderr.write(" Unable to create queue2 \n")
    nvvidconv_postosd = Gst.ElementFactory.make("nvvideoconvert", "convertor_postosd")
    if not nvvidconv_postosd:
        sys.stderr.write(" Unable to create nvvidconv_postosd \n")

    # Create a caps filter
    caps = Gst.ElementFactory.make("capsfilter", "filter")
    caps.set_property("caps", Gst.Caps.from_string("video/x-raw(memory:NVMM), format=I420"))

    # Make the encoder
    if codec == "H264":
        encoder = Gst.ElementFactory.make("nvv4l2h264enc", "encoder")
        print("Creating H264 Encoder")
    elif codec == "H265":
        encoder = Gst.ElementFactory.make("nvv4l2h265enc", "encoder")
        print("Creating H265 Encoder")
    if not encoder:
        sys.stderr.write(" Unable to create encoder")
    encoder.set_property('bitrate', bitrate)
    if is_aarch64():
        encoder.set_property('preset-level', 1)
        encoder.set_property('insert-sps-pps', 1)
        encoder.set_property('bufapi-version', 1)

    # Make the payload-encode video into RTP packets
    if codec == "H264":
        rtppay = Gst.ElementFactory.make("rtph264pay", "rtppay")
        print("Creating H264 rtppay")
    elif codec == "H265":
        rtppay = Gst.ElementFactory.make("rtph265pay", "rtppay")
        print("Creating H265 rtppay")
    if not rtppay:
        sys.stderr.write(" Unable to create rtppay")

    # Make the UDP sink
    updsink_port_num = 5400
    sink = Gst.ElementFactory.make("udpsink", "udpsink")
    if not sink:
        sys.stderr.write(" Unable to create udpsink")

    sink.set_property('host', '224.224.255.255')
    sink.set_property('port', updsink_port_num)
    sink.set_property('async', False)
    sink.set_property('sync', 1)

    if (no_display) :
        print("Creating FakeSink \n")
        sink = Gst.ElementFactory.make("fakesink", "fakesink")
        if not sink:
            sys.stderr.write(" Unable to create fakesink \n")
    else:
        if is_aarch64():
            transform = Gst.ElementFactory.make("nvegltransform", "nvegl-transform")

        print("Creating EGLSink \n")
        sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
        if not sink:
            sys.stderr.write(" Unable to create egl sink \n")

    print("Playing file %s " %input_file)
    source.set_property('location', input_file)
    streammux.set_property('width', 1920)
    streammux.set_property('height', 1080)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)

    pgie.set_property('config-file-path', PGIE_CONFIG_FILE)
    msgconv.set_property('config',MSCONV_CONFIG_FILE)
    msgconv.set_property('payload-type', schema_type)
    msgbroker.set_property('proto-lib', proto_lib)
    msgbroker.set_property('conn-str', conn_str)
    if cfg_file is not None:
        msgbroker.set_property('config', cfg_file)
    if topic is not None:
        msgbroker.set_property('topic', topic)
    msgbroker.set_property('sync', False)

    print("Adding elements to Pipeline \n")
    pipeline.add(source)
    pipeline.add(h264parser)
    pipeline.add(decoder)
    pipeline.add(streammux)
    pipeline.add(pgie)
    pipeline.add(nvvidconv)
    pipeline.add(nvosd)
    pipeline.add(tee)
    pipeline.add(queue1)
    pipeline.add(queue2)
    pipeline.add(msgconv)
    pipeline.add(msgbroker)
    pipeline.add(nvvidconv_postosd)
    pipeline.add(caps)
    pipeline.add(encoder)
    pipeline.add(rtppay)
    pipeline.add(sink)
    if is_aarch64() and not no_display:
        pipeline.add(transform)

    # Need to update!
    # Link the elements together:
    # file-source -> h264-parser -> nvh264-decoder ->
    # nvinfer -> nvvidconv -> nvosd -> nvvidconv_postosd ->
    # caps -> encoder -> rtppay -> udpsink

    print("Linking elements in the Pipeline \n")
    source.link(h264parser)
    h264parser.link(decoder)

    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        sys.stderr.write(" Unable to get the sink pad of streammux \n")
    srcpad = decoder.get_static_pad("src")
    if not srcpad:
        sys.stderr.write(" Unable to get source pad of decoder \n")
    srcpad.link(sinkpad)

    streammux.link(pgie)
    pgie.link(nvvidconv)
    nvvidconv.link(nvosd) ##
    nvosd.link(tee) ##
    nvosd.link(nvvidconv_postosd) ##
    queue1.link(msgconv)
    msgconv.link(msgbroker)
    # if is_aarch64() and not no_display:
    #     queue2.link(transform)
    #     transform.link(sink)
    # else:
    #     queue2.link(sink)
    sink_pad=queue1.get_static_pad("sink")
    tee_msg_pad=tee.get_request_pad('src_%u')
    tee_render_pad=tee.get_request_pad("src_%u")
    if not tee_msg_pad or not tee_render_pad:
        sys.stderr.write("Unable to get request pads\n")
    tee_msg_pad.link(sink_pad)
    sink_pad=queue2.get_static_pad("sink")
    tee_render_pad.link(sink_pad)
#    nvosd.link(nvvidconv_postosd)
    nvvidconv_postosd.link(caps)
    caps.link(encoder)
    encoder.link(rtppay)
    rtppay.link(sink)

    # create an event loop and feed gstreamer bus mesages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect ("message", bus_call, loop)

    # Start streaming
    rtsp_port_num = 8554

    server = GstRtspServer.RTSPServer.new()
    server.props.service = "%d" % rtsp_port_num
    server.attach(None)

    factory = GstRtspServer.RTSPMediaFactory.new()
    factory.set_launch( "( udpsrc name=pay0 port=%d buffer-size=524288 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=(string)%s, payload=96 \" )" % (updsink_port_num, codec))
    factory.set_shared(True)
    server.get_mount_points().add_factory("/ds-test", factory)

    print("\n *** DeepStream: Launched RTSP Streaming at rtsp://localhost:%d/ds-test ***\n\n" % rtsp_port_num)

    # Lets add probe to get informed of the meta data generated, we add probe to
    # the sink pad of the osd element, since by that time, the buffer would have
    # had got all the metadata.
    osdsinkpad = nvosd.get_static_pad("sink")
    if not osdsinkpad:
        sys.stderr.write(" Unable to get sink pad of nvosd \n")

    osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)

    # start play back and listen to events
    print("Starting pipeline \n")

    # start play back and listed to events
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except:
        pass
    # cleanup
    pyds.unset_callback_funcs()
    pipeline.set_state(Gst.State.NULL)

# Parse and validate input arguments
def parse_args():
    parser = OptionParser()
    parser.add_option("-c", "--cfg-file", dest="cfg_file",
                  help="Set the adaptor config file. Optional if connection string has relevant  details.", metavar="FILE")
    parser.add_option("-i", "--input-file", dest="input_file",
                  help="Set the input H264 file", metavar="FILE")
    parser.add_option("-p", "--proto-lib", dest="proto_lib",
                  help="Absolute path of adaptor library", metavar="PATH")
    parser.add_option("", "--conn-str", dest="conn_str",
                  help="Connection string of backend server. Optional if it is part of config file.", metavar="STR")
    parser.add_option("-s", "--schema-type", dest="schema_type", default="0",
                  help="Type of message schema (0=Full, 1=minimal), default=0", metavar="<0|1>")
    parser.add_option("-t", "--topic", dest="topic",
                  help="Name of message topic. Optional if it is part of connection string or config file.", metavar="TOPIC")
    parser.add_option("", "--no-display", action="store_true", dest="no_display", default=False,
                  help="Disable display")
    # parser.add_argument("", "--codec", dest="codec", default="H264",
    #               help="RTSP Streaming Codec H264/H265 , default=H264", metavar="STR")
    # parser.add_argument("-b", "--bitrate",dest="bitrate",  default=4000000,
    #               help="Set the encoding bitrate ", metavar="INT")

    (options, args) = parser.parse_args()

    global cfg_file
    global input_file
    global proto_lib
    global conn_str
    global topic
    global schema_type
    global no_display
    global codec
    global bitrate
    codec = "H264"
    bitrate = 4000000
    cfg_file = options.cfg_file
    input_file = options.input_file
    proto_lib = options.proto_lib
    conn_str = options.conn_str
    topic = options.topic
    no_display = options.no_display

    if proto_lib is None or input_file is None:
        print("Usage: python3 deepstream_test_4.py -i <H264 filename> -p <Proto adaptor library> --conn-str=<Connection string>")
        return 1

    if (options.schema_type == "0"):
        schema_type = 0
    else:
        schema_type = 1

    return 0

if __name__ == '__main__':
    ret = parse_args()
    #If argumer parsing fail, return failure (non-zero)
    if ret == 1:
        sys.exit(1)
    sys.exit(main(sys.argv))