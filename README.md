# link-esp32-example-TG0
This example shows how I am using the Timer Group 0 to call an interrupt function at a tightly timed interval.

Adding changes to the Session State, such as `state.setTempo()` will causes a crash upon calling `link->commitAudioSessionState(sessionState)`

Your most recent commit to the Link API might cause me to rethink using this interrupt as I think you are getting just as tight timing now without having to use an interrupt.
