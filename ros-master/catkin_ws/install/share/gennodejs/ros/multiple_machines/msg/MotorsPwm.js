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

class MotorsPwm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_motor_pwm = null;
      this.right_motor_pwm = null;
    }
    else {
      if (initObj.hasOwnProperty('left_motor_pwm')) {
        this.left_motor_pwm = initObj.left_motor_pwm
      }
      else {
        this.left_motor_pwm = 0;
      }
      if (initObj.hasOwnProperty('right_motor_pwm')) {
        this.right_motor_pwm = initObj.right_motor_pwm
      }
      else {
        this.right_motor_pwm = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorsPwm
    // Serialize message field [left_motor_pwm]
    bufferOffset = _serializer.uint32(obj.left_motor_pwm, buffer, bufferOffset);
    // Serialize message field [right_motor_pwm]
    bufferOffset = _serializer.uint32(obj.right_motor_pwm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorsPwm
    let len;
    let data = new MotorsPwm(null);
    // Deserialize message field [left_motor_pwm]
    data.left_motor_pwm = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [right_motor_pwm]
    data.right_motor_pwm = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'multiple_machines/MotorsPwm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cae0766230de6a370bc83970582495e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 left_motor_pwm
    uint32 right_motor_pwm
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorsPwm(null);
    if (msg.left_motor_pwm !== undefined) {
      resolved.left_motor_pwm = msg.left_motor_pwm;
    }
    else {
      resolved.left_motor_pwm = 0
    }

    if (msg.right_motor_pwm !== undefined) {
      resolved.right_motor_pwm = msg.right_motor_pwm;
    }
    else {
      resolved.right_motor_pwm = 0
    }

    return resolved;
    }
};

module.exports = MotorsPwm;
