package frc.lib.bluecrew.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class CustomAutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {

    public static SendableChooser<Command> buildAutoChooserFromAutosInPPFolder(String defaultAutoName, String pathplannerFolderName) {
        if (!AutoBuilder.isConfigured()) {
            throw new RuntimeException(
                    "AutoBuilder was not configured before attempting to build an auto chooser");
        }

        SendableChooser<Command> chooser = new SendableChooser<>();
        List<String> autoNames = getAllAutoNamesInPPFolder(pathplannerFolderName);

        PathPlannerAuto defaultOption = null;
        List<PathPlannerAuto> options = new ArrayList<>();

        for (String autoName : autoNames) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);

            if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
                defaultOption = auto;
            } else {
                options.add(auto);
            }
        }

        if (defaultOption == null) {
            chooser.setDefaultOption("None", Commands.none());
        } else {
            chooser.setDefaultOption(defaultOption.getName(), defaultOption);
        }

        options.forEach(auto -> chooser.addOption(auto.getName(), auto));

        return chooser;
    }

    public static SendableChooser<Command> buildAutoChooserFromAutosInPPFolder(String pathplannerFolderName) {
        return buildAutoChooserFromAutosInPPFolder("", pathplannerFolderName);
    }

    /**
     * Get a list of all auto names in the specified pathplanner folder
     *
     * @return List of all auto names
     */
    public static List<String> getAllAutoNamesInPPFolder(String pathplannerFolder) {
        List<String> allAutoNames = AutoBuilder.getAllAutoNames();
        List<String> wantedAutoNames = new ArrayList<>();

        for(String autoName : allAutoNames) {
            try (BufferedReader br =
                         new BufferedReader(
                                 new FileReader(
                                         new File(
                                                 Filesystem.getDeployDirectory(), "pathplanner/autos/" + autoName + ".auto")))) {
                StringBuilder fileContentBuilder = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    fileContentBuilder.append(line);
                }

                String fileContent = fileContentBuilder.toString();
                JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

                if(json.get("folder") == pathplannerFolder) {
                    wantedAutoNames.add(autoName);
                }
            } catch (AutoBuilderException e) {
                throw e;
            } catch (Exception e) {
                throw new RuntimeException(e.getMessage());
            }
        }

        return wantedAutoNames;
    }
}
