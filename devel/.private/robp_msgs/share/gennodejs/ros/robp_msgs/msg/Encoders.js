// Auto-generated. Do not edit!

// (in-package robp_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Encoders {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.encoder_left = null;
      this.encoder_right = null;
      this.delta_encoder_left = null;
      this.delta_encoder_right = null;
      this.delta_time_left = null;
      this.delta_time_right = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('encoder_left')) {
        this.encoder_left = initObj.encoder_left
      }
      else {
        this.encoder_left = 0;
      }
      if (initObj.hasOwnProperty('encoder_right')) {
        this.encoder_right = initObj.encoder_right
      }
      else {
        this.encoder_right = 0;
      }
      if (initObj.hasOwnProperty('delta_encoder_left')) {
        this.delta_encoder_left = initObj.delta_encoder_left
      }
      else {
        this.delta_encoder_left = 0;
      }
      if (initObj.hasOwnProperty('delta_encoder_right')) {
        this.delta_encoder_right = initObj.delta_encoder_right
      }
      else {
        this.delta_encoder_right = 0;
      }
      if (initObj.hasOwnProperty('delta_time_left')) {
        this.delta_time_left = initObj.delta_time_left
      }
      else {
        this.delta_time_left = 0.0;
      }
      if (initObj.hasOwnProperty('delta_time_right')) {
        this.delta_time_right = initObj.delta_time_right
      }
      else {
        this.delta_time_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Encoders
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [encoder_left]
    bufferOffset = _serializer.int64(obj.encoder_left, buffer, bufferOffset);
    // Serialize message field [encoder_right]
    bufferOffset = _serializer.int64(obj.encoder_right, buffer, bufferOffset);
    // Serialize message field [delta_encoder_left]
    bufferOffset = _serializer.int32(obj.delta_encoder_left, buffer, bufferOffset);
    // Serialize message field [delta_encoder_right]
    bufferOffset = _serializer.int32(obj.delta_encoder_right, buffer, bufferOffset);
    // Serialize message field [delta_time_left]
    bufferOffset = _serializer.float64(obj.delta_time_left, buffer, bufferOffset);
    // Serialize message field [delta_time_right]
    bufferOffset = _serializer.float64(obj.delta_time_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Encoders
    let len;
    let data = new Encoders(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [encoder_left]
    data.encoder_left = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [encoder_right]
    data.encoder_right = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [delta_encoder_left]
    data.delta_encoder_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [delta_encoder_right]
    data.delta_encoder_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [delta_time_left]
    data.delta_time_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_time_right]
    data.delta_time_right = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robp_msgs/Encoders';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '190037ad3ca3cb2954f783e368d7bd4a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    # Total number of ticks
    int64 encoder_left
    int64 encoder_right
    # The number of ticks since the last reading
    int32 delta_encoder_left
    int32 delta_encoder_right
    # The time elapsed since the last reading in milliseconds
    float64 delta_time_left
    float64 delta_time_right
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
    const resolved = new Encoders(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.encoder_left !== undefined) {
      resolved.encoder_left = msg.encoder_left;
    }
    else {
      resolved.encoder_left = 0
    }

    if (msg.encoder_right !== undefined) {
      resolved.encoder_right = msg.encoder_right;
    }
    else {
      resolved.encoder_right = 0
    }

    if (msg.delta_encoder_left !== undefined) {
      resolved.delta_encoder_left = msg.delta_encoder_left;
    }
    else {
      resolved.delta_encoder_left = 0
    }

    if (msg.delta_encoder_right !== undefined) {
      resolved.delta_encoder_right = msg.delta_encoder_right;
    }
    else {
      resolved.delta_encoder_right = 0
    }

    if (msg.delta_time_left !== undefined) {
      resolved.delta_time_left = msg.delta_time_left;
    }
    else {
      resolved.delta_time_left = 0.0
    }

    if (msg.delta_time_right !== undefined) {
      resolved.delta_time_right = msg.delta_time_right;
    }
    else {
      resolved.delta_time_right = 0.0
    }

    return resolved;
    }
};

module.exports = Encoders;
