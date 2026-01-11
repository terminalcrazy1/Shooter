package frc.robot.util;

import java.util.ArrayList;

public class Signal<EmitMessage> {
    private interface Observer<EmitMessage> {
        public abstract void call(EmitMessage emittedValue);
    }

    private ArrayList<Observer<EmitMessage>> connectedFunctions = new ArrayList<>();

    /** Connects a function or Signal to this Signal. 
     * 
     * If the `target` is a function, it will be called whenever this Signal emits a value.
     * 
     * If the `target` is a Signal, any emitted messages from this Signal will propagate to the target Signal.
     * 
     * @param target The `target` to connect this Signal to
     * @returns A function which will disconnect the `target`
     * */
    public void connect(Observer<EmitMessage> functionToConnect) {
        this.connectedFunctions.add(functionToConnect);
    }

    public void disconnect(Observer<EmitMessage> functionToDisconnect) {
        this.connectedFunctions.remove(functionToDisconnect);
    }

    public void emit(EmitMessage messageToEmit) {
        for(Observer<EmitMessage> functionToCall: this.connectedFunctions) {
            functionToCall.call(messageToEmit);
        }
    }
}