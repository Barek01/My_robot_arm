 let vueApp = new Vue({
    el: "#vueApp",
    data: {

        // ros connection
        ros: null,
        rosbridge_address: 'ws://localhost:9090',
        connected: false,

        // publisher
        pubInterval: null,

        // Axes 

        My_Axes: [
            {Name: "Axis n°1", Joint: "joint_Base_Axe1", Value: 0},
            {Name: "Axis n°2", Joint: "joint_Axe1_Axe2", Value: 0},
            {Name: "Axis n°3", Joint: "joint_Axe2_Axe3", Value: 0},
            {Name: "Axis n°4", Joint: "joint_Axe3_Axe4", Value: 0},
            {Name: "Axis n°5", Joint: "joint_Axe4_Axe5", Value: 0},
            {Name: "Axis n°6", Joint: "joint_Axe5_Axe6", Value: 0},
        ],

        My_Motors_Parameters: [
            {Name: "Axis n°1", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
            {Name: "Axis n°2", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
            {Name: "Axis n°3", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
            {Name: "Axis n°4", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
            {Name: "Axis n°5", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
            {Name: "Axis n°6", Position: 0, Speed: 0, Acceleration: 0, Deceleration: 0, Feedback: 0},
        ]

    },
    methods: {
        connect: function () {
            this.loading = true
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                this.pubInterval = setInterval(this.sendPositionMoteur, 25)
                console.log('Connection to ROSBridge established!')
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                clearInterval(this.pubInterval)
                console.log('Connection to ROSBridge was closed!')
            })
        },
        disconnect: function () {
            this.ros.close()
        },

        sendPositionMoteur: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/web_mvt',
                messageType: 'my_webpage/position_moteurs',
            })
            let message = new ROSLIB.Message({
                name: [ this.My_Axes[0].Joint,
                        this.My_Axes[1].Joint,
                        this.My_Axes[2].Joint,
                        this.My_Axes[3].Joint,
                        this.My_Axes[4].Joint,
                        this.My_Axes[5].Joint],

                position: [ parseFloat(this.My_Axes[0].Value),
                            parseFloat(this.My_Axes[1].Value),
                            parseFloat(this.My_Axes[2].Value),
                            parseFloat(this.My_Axes[3].Value),
                            parseFloat(this.My_Axes[4].Value),
                            parseFloat(this.My_Axes[5].Value)],
            })

            topic.publish(message)
        },

    },

    mounted() {
        // page is ready
        console.log('page is ready!')
    },
})