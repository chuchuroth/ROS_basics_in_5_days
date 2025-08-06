// Auto-generated. Do not edit!

// (in-package wall_following.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class FindWallRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindWallRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindWallRequest
    let len;
    let data = new FindWallRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'wall_following/FindWallRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindWallRequest(null);
    return resolved;
    }
};

class FindWallResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wallfound = null;
    }
    else {
      if (initObj.hasOwnProperty('wallfound')) {
        this.wallfound = initObj.wallfound
      }
      else {
        this.wallfound = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FindWallResponse
    // Serialize message field [wallfound]
    bufferOffset = _serializer.bool(obj.wallfound, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FindWallResponse
    let len;
    let data = new FindWallResponse(null);
    // Deserialize message field [wallfound]
    data.wallfound = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'wall_following/FindWallResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66229cd821efb4126c80f35757f3c73e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool wallfound
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FindWallResponse(null);
    if (msg.wallfound !== undefined) {
      resolved.wallfound = msg.wallfound;
    }
    else {
      resolved.wallfound = false
    }

    return resolved;
    }
};

module.exports = {
  Request: FindWallRequest,
  Response: FindWallResponse,
  md5sum() { return '66229cd821efb4126c80f35757f3c73e'; },
  datatype() { return 'wall_following/FindWall'; }
};
