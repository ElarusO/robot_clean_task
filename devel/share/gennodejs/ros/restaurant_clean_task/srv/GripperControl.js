// Auto-generated. Do not edit!

// (in-package restaurant_clean_task.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GripperControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperControlRequest
    // Serialize message field [angle]
    bufferOffset = _serializer.float64(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperControlRequest
    let len;
    let data = new GripperControlRequest(null);
    // Deserialize message field [angle]
    data.angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'restaurant_clean_task/GripperControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4edb55038e2b888976a0c0c56935341c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 angle  # 爪部目标角度
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperControlRequest(null);
    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    return resolved;
    }
};

class GripperControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.current_angle = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('current_angle')) {
        this.current_angle = initObj.current_angle
      }
      else {
        this.current_angle = 0.0;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperControlResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [current_angle]
    bufferOffset = _serializer.float64(obj.current_angle, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperControlResponse
    let len;
    let data = new GripperControlResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [current_angle]
    data.current_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'restaurant_clean_task/GripperControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd898b2ac5cc8f22ba872cb913242f023';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success   # 执行结果
    float64 current_angle
    string message # 状态信息
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperControlResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.current_angle !== undefined) {
      resolved.current_angle = msg.current_angle;
    }
    else {
      resolved.current_angle = 0.0
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: GripperControlRequest,
  Response: GripperControlResponse,
  md5sum() { return '5813b689e45b3f1ea25d03fd1a946596'; },
  datatype() { return 'restaurant_clean_task/GripperControl'; }
};
