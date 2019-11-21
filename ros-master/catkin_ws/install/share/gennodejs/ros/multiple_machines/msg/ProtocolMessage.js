// Auto-generated. Do not edit!

// (in-package multiple_machines.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ProtocolMessage {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.flag = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = '';
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProtocolMessage
    // Serialize message field [flag]
    bufferOffset = _serializer.string(obj.flag, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.uint32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProtocolMessage
    let len;
    let data = new ProtocolMessage(null);
    // Deserialize message field [flag]
    data.flag = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.flag.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multiple_machines/ProtocolMessage';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b6905edf78d9494d3df19c48997fa57';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string flag
    uint32 value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProtocolMessage(null);
    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = ''
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

module.exports = ProtocolMessage;
