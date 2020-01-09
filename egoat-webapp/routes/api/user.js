const express = require('express');
const router = express.Router();
const User = require('../../models/User');

//Checks if a user exists by username, if they do checks password
router.post('/login', (req, res) => {
    let name = req.body.username;
    let password = req.body.password;
    User.findOne({username: name}).select('-groups -friends -preferences -email -__v').exec(function(err, user) {
    if(!user) {
        res.send({'error' : 'user does not exist'});
    } else{
        res.send({'user': user, 'error': ''});
    }
    });
});

router.get('/get', (req, res) => {
    let name = req.body.username;
    User.findOne({username: name}).exec(function(err, user) {
        if(!user) {
            res.send({'error' : 'user does not exist'});
        } else {
            res.send({'user': user, 'error': ''});
        }
    })
});

//Creates an account with given info
router.post('/createAccount', (req, res) => {
    let username = req.body.username;

    User.findOne({'username': username}).exec(function(err, user) {
        if(!user) {
            let newUser = new User(req.body);
            newUser.save()
            .then(newUser => {
                res.send({'user': newUser, 'error' : ''});
            })
        } else {
            res.send({'error': 'Account with that username already exists'});
        }
    });
});

//Deletes a user by id
router.delete('/deleteAccount/:userId', (req, res) => {
    let userId = req.params.usedId;

    User.findOneAndDelete({id: userId}, function(err, user) {
        if(!user) {
            res.send({error: 'user not found' + err});
        }
        else {
            res.send({'user': user, 'error': ''});
        }
    })
});

module.exports = router;