package frc.robot.util;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

class AutoSelector {
    private Map<String, Command> commandMap;
    private String selectedKey;
    private StringPublisher publisher;
    private StringSubscriber subscriber;

    public AutoSelector(Map<String, Command> commandMap) {
        this.commandMap = commandMap;
        this.selectedKey = null;
        
        NetworkTable table = NetworkTableInstance.getDefault().getTable("AutoSelector");
        publisher = table.getStringTopic("publisher").publish(PubSubOption.keepDuplicates(false));
        subscriber = table.getStringTopic("subscriber").subscribe(null, PubSubOption.keepDuplicates(false));
    }

    public Optional<Command> getSelectedCommand() {
        if (selectedKey != null) {
            return Optional.of(commandMap.get(selectedKey));
        }
        return null;
    }

    public void update() {
        selectedKey = subscriber.get();
        publisher.set(selectedKey);
    }
}