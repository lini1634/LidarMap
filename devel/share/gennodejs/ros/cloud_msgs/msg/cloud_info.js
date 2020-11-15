// Auto-generated. Do not edit!

// (in-package cloud_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class cloud_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.startRingIndex = null;
      this.endRingIndex = null;
      this.startOrientation = null;
      this.endOrientation = null;
      this.orientationDiff = null;
      this.segmentedCloudGroundFlag = null;
      this.segmentedCloudColInd = null;
      this.segmentedCloudRange = null;
      this.XM = null;
      this.Xm = null;
      this.YM = null;
      this.Ym = null;
      this.ZM = null;
      this.Zm = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('startRingIndex')) {
        this.startRingIndex = initObj.startRingIndex
      }
      else {
        this.startRingIndex = [];
      }
      if (initObj.hasOwnProperty('endRingIndex')) {
        this.endRingIndex = initObj.endRingIndex
      }
      else {
        this.endRingIndex = [];
      }
      if (initObj.hasOwnProperty('startOrientation')) {
        this.startOrientation = initObj.startOrientation
      }
      else {
        this.startOrientation = 0.0;
      }
      if (initObj.hasOwnProperty('endOrientation')) {
        this.endOrientation = initObj.endOrientation
      }
      else {
        this.endOrientation = 0.0;
      }
      if (initObj.hasOwnProperty('orientationDiff')) {
        this.orientationDiff = initObj.orientationDiff
      }
      else {
        this.orientationDiff = 0.0;
      }
      if (initObj.hasOwnProperty('segmentedCloudGroundFlag')) {
        this.segmentedCloudGroundFlag = initObj.segmentedCloudGroundFlag
      }
      else {
        this.segmentedCloudGroundFlag = [];
      }
      if (initObj.hasOwnProperty('segmentedCloudColInd')) {
        this.segmentedCloudColInd = initObj.segmentedCloudColInd
      }
      else {
        this.segmentedCloudColInd = [];
      }
      if (initObj.hasOwnProperty('segmentedCloudRange')) {
        this.segmentedCloudRange = initObj.segmentedCloudRange
      }
      else {
        this.segmentedCloudRange = [];
      }
      if (initObj.hasOwnProperty('XM')) {
        this.XM = initObj.XM
      }
      else {
        this.XM = 0.0;
      }
      if (initObj.hasOwnProperty('Xm')) {
        this.Xm = initObj.Xm
      }
      else {
        this.Xm = 0.0;
      }
      if (initObj.hasOwnProperty('YM')) {
        this.YM = initObj.YM
      }
      else {
        this.YM = 0.0;
      }
      if (initObj.hasOwnProperty('Ym')) {
        this.Ym = initObj.Ym
      }
      else {
        this.Ym = 0.0;
      }
      if (initObj.hasOwnProperty('ZM')) {
        this.ZM = initObj.ZM
      }
      else {
        this.ZM = 0.0;
      }
      if (initObj.hasOwnProperty('Zm')) {
        this.Zm = initObj.Zm
      }
      else {
        this.Zm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cloud_info
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [startRingIndex]
    bufferOffset = _arraySerializer.int32(obj.startRingIndex, buffer, bufferOffset, null);
    // Serialize message field [endRingIndex]
    bufferOffset = _arraySerializer.int32(obj.endRingIndex, buffer, bufferOffset, null);
    // Serialize message field [startOrientation]
    bufferOffset = _serializer.float32(obj.startOrientation, buffer, bufferOffset);
    // Serialize message field [endOrientation]
    bufferOffset = _serializer.float32(obj.endOrientation, buffer, bufferOffset);
    // Serialize message field [orientationDiff]
    bufferOffset = _serializer.float32(obj.orientationDiff, buffer, bufferOffset);
    // Serialize message field [segmentedCloudGroundFlag]
    bufferOffset = _arraySerializer.bool(obj.segmentedCloudGroundFlag, buffer, bufferOffset, null);
    // Serialize message field [segmentedCloudColInd]
    bufferOffset = _arraySerializer.uint32(obj.segmentedCloudColInd, buffer, bufferOffset, null);
    // Serialize message field [segmentedCloudRange]
    bufferOffset = _arraySerializer.float32(obj.segmentedCloudRange, buffer, bufferOffset, null);
    // Serialize message field [XM]
    bufferOffset = _serializer.float32(obj.XM, buffer, bufferOffset);
    // Serialize message field [Xm]
    bufferOffset = _serializer.float32(obj.Xm, buffer, bufferOffset);
    // Serialize message field [YM]
    bufferOffset = _serializer.float32(obj.YM, buffer, bufferOffset);
    // Serialize message field [Ym]
    bufferOffset = _serializer.float32(obj.Ym, buffer, bufferOffset);
    // Serialize message field [ZM]
    bufferOffset = _serializer.float32(obj.ZM, buffer, bufferOffset);
    // Serialize message field [Zm]
    bufferOffset = _serializer.float32(obj.Zm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cloud_info
    let len;
    let data = new cloud_info(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [startRingIndex]
    data.startRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [endRingIndex]
    data.endRingIndex = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [startOrientation]
    data.startOrientation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [endOrientation]
    data.endOrientation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationDiff]
    data.orientationDiff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [segmentedCloudGroundFlag]
    data.segmentedCloudGroundFlag = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [segmentedCloudColInd]
    data.segmentedCloudColInd = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    // Deserialize message field [segmentedCloudRange]
    data.segmentedCloudRange = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [XM]
    data.XM = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Xm]
    data.Xm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [YM]
    data.YM = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Ym]
    data.Ym = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ZM]
    data.ZM = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Zm]
    data.Zm = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.startRingIndex.length;
    length += 4 * object.endRingIndex.length;
    length += object.segmentedCloudGroundFlag.length;
    length += 4 * object.segmentedCloudColInd.length;
    length += 4 * object.segmentedCloudRange.length;
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cloud_msgs/cloud_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6297cb853029467c5461fd34cbd847b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header 
    
    int32[] startRingIndex
    int32[] endRingIndex
    
    float32 startOrientation
    float32 endOrientation
    float32 orientationDiff
    
    bool[]    segmentedCloudGroundFlag # true - ground point, false - other points
    uint32[]  segmentedCloudColInd # point column index in range image
    float32[] segmentedCloudRange # point range 
    
    float32 XM
    float32 Xm
    float32 YM
    float32 Ym
    float32 ZM
    float32 Zm
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cloud_info(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.startRingIndex !== undefined) {
      resolved.startRingIndex = msg.startRingIndex;
    }
    else {
      resolved.startRingIndex = []
    }

    if (msg.endRingIndex !== undefined) {
      resolved.endRingIndex = msg.endRingIndex;
    }
    else {
      resolved.endRingIndex = []
    }

    if (msg.startOrientation !== undefined) {
      resolved.startOrientation = msg.startOrientation;
    }
    else {
      resolved.startOrientation = 0.0
    }

    if (msg.endOrientation !== undefined) {
      resolved.endOrientation = msg.endOrientation;
    }
    else {
      resolved.endOrientation = 0.0
    }

    if (msg.orientationDiff !== undefined) {
      resolved.orientationDiff = msg.orientationDiff;
    }
    else {
      resolved.orientationDiff = 0.0
    }

    if (msg.segmentedCloudGroundFlag !== undefined) {
      resolved.segmentedCloudGroundFlag = msg.segmentedCloudGroundFlag;
    }
    else {
      resolved.segmentedCloudGroundFlag = []
    }

    if (msg.segmentedCloudColInd !== undefined) {
      resolved.segmentedCloudColInd = msg.segmentedCloudColInd;
    }
    else {
      resolved.segmentedCloudColInd = []
    }

    if (msg.segmentedCloudRange !== undefined) {
      resolved.segmentedCloudRange = msg.segmentedCloudRange;
    }
    else {
      resolved.segmentedCloudRange = []
    }

    if (msg.XM !== undefined) {
      resolved.XM = msg.XM;
    }
    else {
      resolved.XM = 0.0
    }

    if (msg.Xm !== undefined) {
      resolved.Xm = msg.Xm;
    }
    else {
      resolved.Xm = 0.0
    }

    if (msg.YM !== undefined) {
      resolved.YM = msg.YM;
    }
    else {
      resolved.YM = 0.0
    }

    if (msg.Ym !== undefined) {
      resolved.Ym = msg.Ym;
    }
    else {
      resolved.Ym = 0.0
    }

    if (msg.ZM !== undefined) {
      resolved.ZM = msg.ZM;
    }
    else {
      resolved.ZM = 0.0
    }

    if (msg.Zm !== undefined) {
      resolved.Zm = msg.Zm;
    }
    else {
      resolved.Zm = 0.0
    }

    return resolved;
    }
};

module.exports = cloud_info;
