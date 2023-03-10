package frc.robot.CSE;

import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A utility class used to interface with the parameters for commands from cougar scripts.
 * Enables access of the parameters from JSON format.
 * 
 * @author Brandon C.
 */
public class CougarScriptObject {

    private JSONObject parameters;

    public CougarScriptObject(JSONObject parameters) {
        this.parameters = parameters;
    }

    /**
     * Gets a double with the given key.
     * @param key The key/name of the value in the parameters.
     * @return The double stored in the parameters.
     */
    public double getDouble(String key) {
        return parameters.getDouble(key);
    }

    /**
     * Gets an angle with the given key.
     * @param key The key/name of the value in the parameters.
     * @return The angle as a Rotation2d. The angle should be stored in degrees in the JSON file.
     */
    public Rotation2d getAngle(String key) {
        return new Rotation2d(Math.toRadians(getDouble(key)));
    }

    /**
     * Gets a JSONArray with the given key.
     * @param key The key/name of the value in the parameters.
     * @return The JSONArray stored in the parameters.
     */
    public JSONArray getJSONArray(String key) {
        return parameters.getJSONArray(key);
    }

    /**
     * Gets a list of points with the given key.
     * Points are stored in the form of a JSONArray of JSONObjects, each one with an x and y double property.
     * Example: Points:[{"x":10.5, "y":20.4}, {"x":13.25, "y":19.5}, {"x":3.33, "y":1.125}
     * getPointList("Points") -> List.of(new Translation2d(10.5, 20.4), ...)
     * @param key The key/name of the value in the parameters.
     * @return The list of points stored in the parameters as a list<Translation2d>.
     */
    public List<Translation2d> getPointList(String key) {
        JSONArray pointsJSON = parameters.getJSONArray(key);
        List<Translation2d> points = new ArrayList<>();
            for (int i = 0; i < pointsJSON.length(); i++) {
                JSONObject json = pointsJSON.getJSONObject(i);
                points.add(
                    new Translation2d(
                        json.getDouble("x"),
                        json.getDouble("y")
                    )
                );
            }
            return points;
    }

    /**
     * Gets a boolean with the given key.
     * @param key The key/name of the value in the parameters.
     * @return The boolean stored in the parameters.
     */
    public boolean getBoolean(String key) {
        return parameters.getBoolean(key);
    }
}