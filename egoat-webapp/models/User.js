const mongoose = require('mongoose');
const Schema = mongoose.Schema;
const EventScheme = require('./Event');

let UserSchema = new Schema({
    email: {
        type: String
    },
    username : {
        type: String
    },
    password: {
        type: String
    },
    events: {
        type: [EventScheme]
    }
    
});

module.exports = User = mongoose.model('user', UserSchema);