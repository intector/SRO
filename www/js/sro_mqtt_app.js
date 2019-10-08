
$(document).ready(function() {
  websocketclient.connect();
});

/* ESP32 life bit handling */
var LifeBitActive = null;

function LifeBitOn(){
  LifeBitActive = setInterval(function(){
    websocketclient.publish(MQTT_PublishTag.lifebit_out, "life bit", 0, false);
//    console.log(MQTT_PublishTag.lifebit_out);
  },1000);   
}

const tod = document.querySelector('div.conn_esp32');
var _timeout = null;

function TO_Start(){
    _timeout = setTimeout(function() {
      cmdButtonsDisable(true);
      tod.classList.remove('connected');
      tod.classList.add('notconnected');
      $('.TempValue').removeClass('TempSensor_ERR').removeClass('TempSensor_OK').addClass('TempSensor_OFF');
      ActualTemperature.sTemp_C = "0";
      document.getElementById("TempDisplay").innerHTML = ActualTemperature.sTemp_C + datTempUnit;
    }, 4000);
}

function RestartTimer(){
  clearTimeout(_timeout);
  cmdButtonsDisable(false);
  tod.classList.remove('notconnected');
  tod.classList.add('connected');
  TO_Start();
}

// variable to hold button pointer
var _ActAniBtn = null;

function cmdBtn_ClearClasses(_CMD_BTN){
  _CMD_BTN.classList.remove('startup');
  _CMD_BTN.classList.remove('sroRed');
  _CMD_BTN.classList.remove('sroYellow');
  _CMD_BTN.classList.remove('sroGreen');
  _CMD_BTN.classList.remove('sroBtnGray');
}

var websocketclient = {
    'client': null,
    'lastMessageId': 1,
    'lastSubId': 1,
    'subscriptions': [],
    'messages': [],
    'connected': false,

    'connect': function () {

        var host = mqtt_conf_host;
        var port = mqtt_conf_port;
        var clientId = mqtt_conf_clientId;
        var username = mqtt_conf_username;
        var password = mqtt_conf_password;

        var keepAlive = 120;
        var cleanSession = true;
        var lwTopic = "/a2s_lw/LWT";
        var lwQos = 0;
        var lwRetain = false;
        var lwMessage = "That's the LW message";
        var ssl = true;

        this.client = new Messaging.Client(host, port, clientId);
        this.client.onConnectionLost = this.onConnectionLost;
        this.client.onMessageArrived = this.onMessageArrived;

        var options = {
            timeout: 3,
            keepAliveInterval: keepAlive,
            cleanSession: cleanSession,
            useSSL: ssl,
            onSuccess: this.onConnect,
            onFailure: this.onFail
        };

        if (username.length > 0) {
            options.userName = username;
        }
        if (password.length > 0) {
            options.password = password;
        }
        if (lwTopic.length > 0) {
            var willmsg = new Messaging.Message(lwMessage);
            willmsg.qos = lwQos;
            willmsg.destinationName = lwTopic;
            willmsg.retained = lwRetain;
            options.willMessage = willmsg;
        }

        this.client.connect(options);
    },

    'onConnect': function () {
      websocketclient.connected = true;
      console.log("mqtt connected...");
      $('.conn_mqtt').addClass('connected').removeClass('notconnected').removeClass('connectionbroke');

      // start lifebit monitoring
      TO_Start();
      LifeBitOn();
      
      websocketclient.subscribe(MQTT_SubscriptionTag.temperature, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.subSolParLoad, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.subPID_ParLoad, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.run_in, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.heater_in, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.fan_in, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.door_in, 0);
      websocketclient.subscribe(MQTT_SubscriptionTag.lifebit_in, 0);

      // get controller data and CMD status on MQTT connect
      websocketclient.publish(MQTT_PublishTag.data_update_out, "MQTT-Connect", 0, false);
    },

    'onFail': function (message) {
        websocketclient.connected = false;
        console.log("error: " + message.errorMessage);
        websocketclient.render.showError('Connect failed: ' + message.errorMessage);
    },

    'onConnectionLost': function (responseObject) {
        websocketclient.connected = false;
        if (responseObject.errorCode !== 0) {
//          console.log("onConnectionLost:" + responseObject.errorMessage);
        }
//        $('#connectionStatus.connected').removeClass('connected').addClass('notconnected').addClass('connectionbroke');
        $('.conn_mqtt').removeClass('connected').addClass('notconnected').addClass('connectionbroke');
        clearInterval(LifeBitActive);

        //Cleanup messages
        websocketclient.messages = [];
        websocketclient.render.clearMessages();

        //Cleanup subscriptions
        websocketclient.subscriptions = [];
        websocketclient.render.clearSubscriptions();
    },

    'onMessageArrived': function (message) {
      if (message.destinationName == MQTT_SubscriptionTag.lifebit_in ){
        RestartTimer();
      }

      if (message.destinationName == MQTT_SubscriptionTag.subSolParLoad ){
        SolderingParameter = JSON.parse(message.payloadString);
        console.log("answer: " + message.payloadString);
        document.getElementById("idSRO_ParSetSel").selectedIndex = SolderingParameter.idSRO_ParSetSel - 1;
        document.getElementById("InputTime01").value = SolderingParameter.InputTime01;
        document.getElementById("InputTime02").value = SolderingParameter.InputTime02;
        document.getElementById("InputTime03").value = SolderingParameter.InputTime03;
        document.getElementById("InputTime04").value = SolderingParameter.InputTime04;
        document.getElementById("InputTime05").value = SolderingParameter.InputTime05;
        document.getElementById("InputTemp01").value = SolderingParameter.InputTemp01;
        document.getElementById("InputTemp02").value = SolderingParameter.InputTemp02;
        document.getElementById("InputTemp03").value = SolderingParameter.InputTemp03;
        document.getElementById("InputTemp04").value = SolderingParameter.InputTemp04;
        document.getElementById("InputTemp05").value = SolderingParameter.InputTemp05;
        SRO_SetValChart_Update();
      }

      if (message.destinationName == MQTT_SubscriptionTag.subPID_ParLoad ){
        ParPID = JSON.parse(message.payloadString);
        document.getElementById("maint_PID_KP").value = ParPID.PID_KP;
        document.getElementById("maint_PID_KI").value = ParPID.PID_KI;
        document.getElementById("maint_PID_KD").value = ParPID.PID_KD;
      }

      if (message.destinationName == MQTT_SubscriptionTag.temperature ){
        ActualTemperature = JSON.parse(message.payloadString);
        _tmpTempDisplay = document.getElementById("TempDisplay");
        if (ActualTemperature.bSensor_OK){
          $('.TempValue').removeClass('TempSensor_OFF').removeClass('TempSensor_ERR').addClass('TempSensor_OK');
          _tmpTempDisplay.innerHTML = ActualTemperature.sTemp_C + datTempUnit;
        } else{
          $('.TempValue').removeClass('TempSensor_OFF').removeClass('TempSensor_OK').addClass('TempSensor_ERR');
          _tmpTempDisplay.innerHTML = "error";
        }
        DataStream();
      }

      if (message.destinationName == MQTT_SubscriptionTag.run_in ){
        CTRL_CMD_VALUE = JSON.parse(message.payloadString);
        _ActAniBtn = document.getElementById("btnSolderingSequence");
        cmdBtn_ClearClasses(_ActAniBtn);
        if (CTRL_CMD_VALUE.SEQ_RUN){
          _ActAniBtn.classList.add('running');
          _ActAniBtn.classList.add('sroGreen');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txton;
        } else{
          _ActAniBtn.classList.add('sroBtnGray');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txtoff;
        }
      }

      if (message.destinationName == MQTT_SubscriptionTag.heater_in ){
        CTRL_CMD_VALUE = JSON.parse(message.payloadString);
        _ActAniBtn = document.getElementById("btnHeatingElements");
        cmdBtn_ClearClasses(_ActAniBtn);
        if (CTRL_CMD_VALUE.HEATER){
          _ActAniBtn.classList.add('running');
          _ActAniBtn.classList.add('sroGreen');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txton;
        } else{
          _ActAniBtn.classList.add('sroBtnGray');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txtoff;
        }
      }

      if (message.destinationName == MQTT_SubscriptionTag.fan_in ){
        CTRL_CMD_VALUE = JSON.parse(message.payloadString);
        _ActAniBtn = document.getElementById("btnConvectionFan");
        cmdBtn_ClearClasses(_ActAniBtn);
        if (CTRL_CMD_VALUE.FAN){
          _ActAniBtn.classList.add('running');
          _ActAniBtn.classList.add('sroGreen');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txton;
        } else{
          _ActAniBtn.classList.add('sroBtnGray');
          _ActAniBtn.textContent = _ActAniBtn.dataset.txtoff;
        }
      }

      if (message.destinationName == MQTT_SubscriptionTag.door_in ){
        CTRL_CMD_VALUE = JSON.parse(message.payloadString);
        _ActAniBtn = document.getElementById("btnDoor");
        _ActAniBtn.value = CTRL_CMD_VALUE.DOOR;
      }

    },

    'disconnect': function () {
        this.client.disconnect();
    },

    'publish': function (topic, payload, qos, retain) {

        if (!websocketclient.connected) {
            websocketclient.render.showError("Not connected on publish");
            return false;
        }

        var message = new Messaging.Message(payload);
        message.destinationName = topic;
        message.qos = qos;
        message.retained = retain;
        this.client.send(message);
    },

    'subscribe': function (topic, qosNr) {

        if (!websocketclient.connected) {
            websocketclient.render.showError("Not connected on subscribe");
            return false;
        }

        this.client.subscribe(topic, qosNr);

        return true;
    },

    'unsubscribe': function (id) {
        var subs = _.find(websocketclient.subscriptions, {'id': id});
        this.client.unsubscribe(subs.topic);
        websocketclient.subscriptions = _.filter(websocketclient.subscriptions, function (item) {
            return item.id != id;
        });

        websocketclient.render.removeSubscriptionsMessages(id);
    },

    'deleteSubscription': function (id) {
        var elem = $("#sub" + id);

        if (confirm('Are you sure ?')) {
            elem.remove();
            this.unsubscribe(id);
        }
    },
/*
    'getRandomColor': function () {
        var r = (Math.round(Math.random() * 255)).toString(16);
        var g = (Math.round(Math.random() * 255)).toString(16);
        var b = (Math.round(Math.random() * 255)).toString(16);
        return r + g + b;
    },

    'getSubscriptionForTopic': function (topic) {
        var i;
        for (i = 0; i < this.subscriptions.length; i++) {
            if (this.compareTopics(topic, this.subscriptions[i].topic)) {
                return this.subscriptions[i];
            }
        }
        return false;
    },

    'getColorForPublishTopic': function (topic) {
        var id = this.getSubscriptionForTopic(topic);
        return this.getColorForSubscription(id);
    },

    'getColorForSubscription': function (id) {
        try {
            if (!id) {
                return '99999';
            }

            var sub = _.find(this.subscriptions, { 'id': id });
            if (!sub) {
                return '999999';
            } else {
                return sub.color;
            }
        } catch (e) {
            return '999999';
        }
    },

    'compareTopics': function (topic, subTopic) {
        var pattern = subTopic.replace("+", "(.*?)").replace("#", "(.*)");
        var regex = new RegExp("^" + pattern + "$");
        return regex.test(topic);
    },
*/
    'render': {

        'showError': function (message) {
            alert(message);
        },
      
/*
        'messages': function () {

            websocketclient.render.clearMessages();
            _.forEach(websocketclient.messages, function (message) {
                message.id = websocketclient.render.message(message);
            });

        },
        'message': function (message) {

            var largest = websocketclient.lastMessageId++;

            var html = '<li class="messLine id="' + largest + '">' +
                '   <div class="row large-12 mess' + largest + '" style="border-left: solid 10px #' + message.color + '; ">' +
                '       <div class="large-12 columns messageText">' +
                '           <div class="large-3 columns date">' + message.timestamp.format("YYYY-MM-DD HH:mm:ss") + '</div>' +
                '           <div class="large-5 columns topicM truncate" id="topicM' + largest + '" title="' + Encoder.htmlEncode(message.topic, 0) + '">Topic: ' + Encoder.htmlEncode(message.topic) + '</div>' +
                '           <div class="large-2 columns qos">Qos: ' + message.qos + '</div>' +
                '           <div class="large-2 columns retain">';
            if (message.retained) {
                html += 'Retained';
            }
            html += '           </div>' +
                '           <div class="large-12 columns message break-words">' + Encoder.htmlEncode(message.payload) + '</div>' +
                '       </div>' +
                '   </div>' +
                '</li>';
            $("#messEdit").prepend(html);
            return largest;
        },
*/
/*
        'subscriptions': function () {
            websocketclient.render.clearSubscriptions();
            _.forEach(websocketclient.subscriptions, function (subs) {
                subs.id = websocketclient.render.subscription(subs);
            });
        },

        'subscription': function (subscription) {
            var largest = websocketclient.lastSubId++;
            $("#innerEdit").append(
                '<li class="subLine" id="sub' + largest + '">' +
                    '   <div class="row large-12 subs' + largest + '" style="border-left: solid 10px #' + subscription.color + '; background-color: #ffffff">' +
                    '       <div class="large-12 columns subText">' +
                    '           <div class="large-1 columns right closer">' +
                    '              <a href="#" onclick="websocketclient.deleteSubscription(' + largest + '); return false;">x</a>' +
                    '           </div>' +
                    '           <div class="qos">Qos: ' + subscription.qos + '</div>' +
                    '           <div class="topic truncate" id="topic' + largest + '" title="' + Encoder.htmlEncode(subscription.topic, 0) + '">' + Encoder.htmlEncode(subscription.topic) + '</div>' +
                    '       </div>' +
                    '   </div>' +
                    '</li>');
            return largest;
        },
*/
/*
        'toggleAll': function () {
            websocketclient.render.toggle('conni');
            websocketclient.render.toggle('publish');
            websocketclient.render.toggle('messages');
            websocketclient.render.toggle('sub');
        },

        'toggle': function (name) {
            $('.' + name + 'Arrow').toggleClass("closed");
            $('.' + name + 'Top').toggleClass("closed");
            var elem = $('#' + name + 'Main');
            elem.slideToggle();
        },

        'hide': function (name) {
            $('.' + name + 'Arrow').addClass("closed");
            $('.' + name + 'Top').addClass("closed");
            var elem = $('#' + name + 'Main');
            elem.slideUp();
        },

        'show': function (name) {
            $('.' + name + 'Arrow').removeClass("closed");
            $('.' + name + 'Top').removeClass("closed");
            var elem = $('#' + name + 'Main');
            elem.slideDown();
        },

        'removeSubscriptionsMessages': function (id) {
            websocketclient.messages = _.filter(websocketclient.messages, function (item) {
                return item.subscriptionId != id;
            });
            websocketclient.render.messages();
        },

        'clearMessages': function () {
            $("#messEdit").empty();
        },

        'clearSubscriptions': function () {
            $("#innerEdit").empty();
        }
*/
    }
};

