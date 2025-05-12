// Auto-generated. Do not edit!

// (in-package dm_motor_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MITCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kp = null;
      this.kd = null;
      this.position = null;
      this.velocity = null;
      this.torque = null;
    }
    else {
      if (initObj.hasOwnProperty('kp')) {
        this.kp = initObj.kp
      }
      else {
        this.kp = 0.0;
      }
      if (initObj.hasOwnProperty('kd')) {
        this.kd = initObj.kd
      }
      else {
        this.kd = 0.0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0.0;
      }
      if (initObj.hasOwnProperty('torque')) {
        this.torque = initObj.torque
      }
      else {
        this.torque = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MITCommand
    // Serialize message field [kp]
    bufferOffset = _serializer.float64(obj.kp, buffer, bufferOffset);
    // Serialize message field [kd]
    bufferOffset = _serializer.float64(obj.kd, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.float64(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.float64(obj.velocity, buffer, bufferOffset);
    // Serialize message field [torque]
    bufferOffset = _serializer.float64(obj.torque, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MITCommand
    let len;
    let data = new MITCommand(null);
    // Deserialize message field [kp]
    data.kp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [kd]
    data.kd = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque]
    data.torque = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dm_motor_controller/MITCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8ffcd33c7fb2269670e5e49cb5a27d69';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 kp
    float64 kd
    float64 position
    float64 velocity
    float64 torque
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MITCommand(null);
    if (msg.kp !== undefined) {
      resolved.kp = msg.kp;
    }
    else {
      resolved.kp = 0.0
    }

    if (msg.kd !== undefined) {
      resolved.kd = msg.kd;
    }
    else {
      resolved.kd = 0.0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0.0
    }

    if (msg.torque !== undefined) {
      resolved.torque = msg.torque;
    }
    else {
      resolved.torque = 0.0
    }

    return resolved;
    }
};

module.exports = MITCommand;
