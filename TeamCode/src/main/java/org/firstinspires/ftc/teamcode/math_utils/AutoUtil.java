package org.firstinspires.ftc.teamcode.math_utils;

import java.util.Arrays;
import java.util.concurrent.Callable;

public class AutoUtil {

    public enum AutoActionState {
        IDLE,
        RUNNING,
        FINISHED
    }

    static AutoActionState[] loopStates;

    /**
     * Runs a list of instantaneous methods in parallel.
     * Methods used must return an AutoActionState immediately after being called.
     * This type represents whether the last call of the method completed a desired action.
     * @param actions The list of methods to run, as an array of Callables.
     */
    public static void runActionsConcurrent(Callable<AutoActionState>[] actions) {
        loopStates = new AutoActionState[actions.length];
        Arrays.fill(loopStates, AutoActionState.IDLE);
        while (!Arrays.stream(loopStates).allMatch(state -> state == AutoActionState.FINISHED)) {
            for (int i = 0; i < actions.length; i++) {
                    if (loopStates[i] != AutoActionState.FINISHED) {
                            try { loopStates[i] = actions[i].call(); }
                            catch (Exception e) { throw new RuntimeException(e); }
                    }

            }
        }

    }
    /**
     * Returns an indexed multi-line readout of the AutoActionStates for the running actions.
     * @return The current state readout
     */
    public static String getLoopStatesReadout() {
        return toString(loopStates);
    }

    /**
     * Returns an indexed multi-line readout of a provided array of AutoActionStates
     * @param states The list of states to create the readout with
     * @return The state readout
     */
    public static String toString(AutoActionState[] states) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < states.length; i++) {
            builder.append(i);
            builder.append(": ");
            builder.append(toString(states[i]));
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    /**
     * Returns a string representation of an AutoActionState
     * @param state The state to be converted
     * @return The string representation
     */
    public static String toString(AutoActionState state) {
        switch (state) {
            case RUNNING:
                return "RUNNING";
            case FINISHED:
                return "FINISHED";
            default:
                return "IDLE";
        }
    }

    /**
     * Gets the AutoActionStates for the running actions.
     * @return The AutoActionStates
     */
    public static AutoActionState[] getLoopStates() { return loopStates; }

}
