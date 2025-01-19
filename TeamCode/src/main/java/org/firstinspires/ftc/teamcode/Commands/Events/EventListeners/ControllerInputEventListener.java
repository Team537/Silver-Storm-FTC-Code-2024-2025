package org.firstinspires.ftc.teamcode.Commands.Events.EventListeners;

import java.util.function.Supplier;

final public class ControllerInputEventListener implements EventListener {

    private Supplier<Boolean> currentControllerInputSupplier;
    private ControllerInputEventType controllerInputEventType;

    // Storage
    private boolean previousStepActive;

    /**
     * Creates a new ControllerInputEventListener for the given controller Input.
     *
     * @param currentControllerInputSupplier A supplier that provides this event with the current controller input.
     */
    public ControllerInputEventListener(Supplier<Boolean> currentControllerInputSupplier) {
        this.currentControllerInputSupplier = currentControllerInputSupplier;
        this.controllerInputEventType = ControllerInputEventType.WHILE_ACTIVE;
    }

    /**
     * Creates a new ControllerInputEventListener for the given controller Input.
     *
     * @param getCurrentControllerInput A supplier that provides this event with the current controller input.
     * @param controllerInputEventType Governs how the controller input will be governed.
     */
    public ControllerInputEventListener(Supplier<Boolean> getCurrentControllerInput, ControllerInputEventType controllerInputEventType) {
        this.currentControllerInputSupplier = getCurrentControllerInput;
        this.controllerInputEventType = controllerInputEventType;
    }

    /**
     * Returns whether or not the event is active. In this case, this depends on the set controller input type,
     * which governs how the controller input is interpreted.
     *
     * @return Whether or not the event is active.
     */
    @Override
    public boolean isEventActive() {

        // Initialize the default state of eventActive as false
        boolean isEventActive = false;

        // Get the current state of the controller input
        boolean currentInputState = currentControllerInputSupplier.get();

        // Determine the event type and update isEventActive accordingly
        switch (controllerInputEventType) {
            case WHILE_ACTIVE:

                // Event is active while the input is currently active
                isEventActive = currentInputState;
                break;
            case RISING_EDGE:

                // Event is active if the input has transitioned from inactive to active
                isEventActive = currentInputState && !previousStepActive;
                break;
            case FALLING_EDGE:

                // Event is active if the input has transitioned from active to inactive
                isEventActive = !currentInputState && previousStepActive;
                break;
            default:

                // Handle unexpected event types (optional but good practice)
                throw new IllegalArgumentException("Unknown controller input event type: " + controllerInputEventType);
        }

        return isEventActive;
    }
}
