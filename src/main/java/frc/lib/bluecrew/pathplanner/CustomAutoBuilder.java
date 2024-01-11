package frc.lib.bluecrew.pathplanner;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class CustomAutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {

    /**
     * Get a list of all auto names in the specified pathplanner folder
     *
     * @return List of all auto names
     */
    public static List<String> getAllAutoNames(String pathplannerFolder) {
        File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

        if (autoFiles == null) {
            return new ArrayList<>();
        }

        return Stream.of(autoFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".auto"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }
}
