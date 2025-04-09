package frc.robot.util;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AdjustableValues {
    private static HashMap<String, Boolean> hasChanged = new HashMap<String, Boolean>();

    private static HashMap<String,LoggedNetworkBoolean> loggedNetworkBooleans = new HashMap<String,LoggedNetworkBoolean>();
    private static HashMap<String,LoggedNetworkString> loggedNetworkStrings = new HashMap<String,LoggedNetworkString>();
    private static HashMap<String,LoggedNetworkNumber> loggedNetworkNumbers = new HashMap<String,LoggedNetworkNumber>();
    private static HashMap<String,LoggedDashboardChooser<?>> loggedDashboardChoosers = new HashMap<String,LoggedDashboardChooser<?>>();
    private static HashMap<String,Boolean> loggedBooleans = new HashMap<String,Boolean>();
    private static HashMap<String,String> loggedStrings = new HashMap<String,String>();
    private static HashMap<String,Double> loggedNumbers = new HashMap<String,Double>();
    private static HashMap<String,Object> loggedChoosers = new HashMap<String,Object>();

    /**
     * Adds a boolean value to the logged values.
     * The default return value is false.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey     The NetworkTables key for logging.
     * @param aliases   Any alternate keys to read the value with. They have their
     *                  own entry in the hasChanged table, and don't affect the
     *                  status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerBoolean(String shortName, String ntKey, String... aliases) {
        return registerBoolean(shortName, ntKey, false, aliases);
    }

    /**
     * Adds a boolean value to the logged values.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName    The short name to get the value with.
     * @param ntKey        The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases      Any alternate keys to read the value with. They have
     *                     their own entry in the hasChanged table, and don't affect
     *                     the status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerBoolean(String shortName, String ntKey, boolean defaultValue, String... aliases) {
        if (hasChanged.containsKey(shortName))
            return false;

        LoggedNetworkBoolean loggedBool = new LoggedNetworkBoolean(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (hasChanged.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkBooleans.put(aliases[i], loggedBool);
            loggedBooleans.put(aliases[i], defaultValue);
            hasChanged.put(aliases[i], true);
        }

        loggedNetworkBooleans.put(shortName, loggedBool);
        loggedBooleans.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds a double value to the logged values.
     * The default return value is 0.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey     The NetworkTables key for logging.
     * @param aliases   Any alternate keys to read the value with. They have their
     *                  own entry in the hasChanged table, and don't affect the
     *                  status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerNumber(String shortName, String ntKey, String... aliases) {
        return registerNumber(shortName, ntKey, 0, aliases);
    }

    /**
     * Adds a double value to the logged values.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName    The short name to get the value with.
     * @param ntKey        The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases      Any alternate keys to read the value with. They have
     *                     their own entry in the hasChanged table, and don't affect
     *                     the status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerNumber(String shortName, String ntKey, double defaultValue, String... aliases) {
        if (hasChanged.containsKey(shortName))
            return false;

        LoggedNetworkNumber loggedNum = new LoggedNetworkNumber(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (hasChanged.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkNumbers.put(aliases[i], loggedNum);
            loggedNumbers.put(aliases[i], defaultValue);
            hasChanged.put(aliases[i], true);
        }

        loggedNetworkNumbers.put(shortName, loggedNum);
        loggedNumbers.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds a string value to the logged values.
     * The default return value is an empty string.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey     The NetworkTables key for logging.
     * @param aliases   Any alternate keys to read the value with. They have their
     *                  own entry in the hasChanged table, and don't affect the
     *                  status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerString(String shortName, String ntKey, String[] aliases) {
        return registerString(shortName, ntKey, "", aliases);
    }

    /**
     * Adds a string value to the logged values.
     * 
     * If any aliases already exist, then the function returns false and no aliases
     * are created.
     * 
     * @param shortName    The short name to get the value with.
     * @param ntKey        The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases      Any alternate keys to read the value with. They have
     *                     their own entry in the hasChanged table, and don't affect
     *                     the status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerString(String shortName, String ntKey, String defaultValue, String... aliases) {
        if (hasChanged.containsKey(shortName))
            return false;

        LoggedNetworkString loggedStr = new LoggedNetworkString(ntKey, defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedNetworkNumber for each alias.
            if (hasChanged.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedNetworkStrings.put(aliases[i], loggedStr);
            loggedStrings.put(aliases[i], defaultValue);
            hasChanged.put(aliases[i], true);
        }

        loggedNetworkStrings.put(shortName, loggedStr);
        loggedStrings.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds a {@link SendableChooser} to the logged values.
     * The default return value is an empty string.
     * 
     * If any aliases already exist, then the function returns false and no aliases are created.
     * 
     * @param shortName The short name to get the value with.
     * @param ntKey     The NetworkTables key for logging.
     * @param aliases   Any alternate keys to read the value with. They have their
     *                  own entry in the hasChanged table, and don't affect the
     *                  status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static boolean registerChooser(String shortName, String ntKey, String[] aliases) {
        return registerChooser(shortName, ntKey, "", aliases);
    }

    /**
     * Adds a {@link SendableChooser} to the logged values.
     * 
     * If any aliases already exist, then the function returns false and no aliases are created.
     * 
     * @param shortName    The short name to get the value with.
     * @param ntKey        The NetworkTables key for logging.
     * @param defaultValue The default value to retrieve from the AKit Logger.
     * @param aliases      Any alternate keys to read the value with. They have
     *                     their own entry in the hasChanged table, and don't affect
     *                     the status of the original shortName.
     * 
     * @return Returns false if the value already exists.
     */
    public static <T> boolean registerChooser(String shortName, String ntKey, T defaultValue, String... aliases) {
        if (hasChanged.containsKey(shortName))
            return false;

        LoggedDashboardChooser<T> chooser = new LoggedDashboardChooser<T>(ntKey);
        chooser.addDefaultOption("default", defaultValue);

        for (int i = 0; i < aliases.length; i++) {
            // Rather than use recursion, I manually put in the aliases so I don't create a
            // new LoggedDashboardChooser for each alias.
            if (hasChanged.containsKey(aliases[i])) {
                for (int j = 0; j < i; j++) {
                    remove(aliases[j]);
                }

                return false;
            }

            loggedDashboardChoosers.put(aliases[i], chooser);
            hasChanged.put(aliases[i], true);
        }

        loggedDashboardChoosers.put(shortName, chooser);
        loggedChoosers.put(shortName, defaultValue);
        hasChanged.put(shortName, true);

        return true;
    }

    /**
     * Adds an alias for the provided shortName.
     * 
     * @param shortName The source for the number
     * @param alias     The alias to add.
     * 
     * @return Returns false if the alias already exists or if the shortName doesn't
     *         exist.
     */
    public static boolean addAlias(String shortName, String alias) {
        if (hasChanged.containsKey(alias) || !hasChanged.containsKey(shortName))
            return false;

        // Checks if the shortName is a boolean
        if (loggedBooleans.containsKey(shortName)) {
            loggedNetworkBooleans.put(alias, loggedNetworkBooleans.get(shortName));
            loggedBooleans.put(alias, loggedNetworkBooleans.get(shortName).get());
            hasChanged.put(alias, true);
            return true;
        }

        // Checks if the shortName is a number
        if (loggedNumbers.containsKey(shortName)) {
            loggedNetworkNumbers.put(alias, loggedNetworkNumbers.get(shortName));
            loggedNumbers.put(alias, loggedNetworkNumbers.get(shortName).get());
            hasChanged.put(alias, true);
            return true;
        }

        // Checks if the shortName is a String
        if (loggedStrings.containsKey(shortName)) {
            loggedNetworkStrings.put(alias, loggedNetworkStrings.get(shortName));
            loggedStrings.put(alias, loggedNetworkStrings.get(shortName).get());
            hasChanged.put(alias, true);
            return true;
        }

        // This theoretically shouldn't run, but it's here just in case
        return false;
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns false.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static boolean getBoolean(String shortName) {
        if (!loggedBooleans.containsKey(shortName))
            return false;

        hasChanged.put(shortName, false);

        return loggedBooleans.get(shortName);
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns 0.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static double getNumber(String shortName) {
        if (!loggedNumbers.containsKey(shortName))
            return 0;

        hasChanged.put(shortName, false);

        return loggedNumbers.get(shortName);
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns an empty string.
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static String getString(String shortName) {
        if (!loggedStrings.containsKey(shortName))
            return "";

        hasChanged.put(shortName, false);

        return loggedStrings.get(shortName);
    }

    /**
     * Gets a value from the logger and marks it as read.
     * 
     * If the key has not been created, it returns null.
     * 
     * @param shortName The first parameter from the register() function.
     */
    @SuppressWarnings("unchecked")
    public static <T> T getChooser(String shortName) {
        if (!loggedStrings.containsKey(shortName)) return null;

        hasChanged.put(shortName, false);

        return (T) loggedChoosers.get(shortName);
    }

    /**
     * Gets whether or not the value has changed.
     * 
     * If the key has not been created, it returns "".
     * 
     * @param shortName The first parameter from the register() function.
     */
    public static boolean hasChanged(String shortName) {
        if (!hasChanged.containsKey(shortName))
            return false;

        return hasChanged.get(shortName);
    }

    /**
     * Updates the values with their current value in NetworkTables and updates the
     * corresponding hasChanged value accordingly.
     */
    public static void updateValues() {
        for (String shortName : loggedNetworkBooleans.keySet()) {
            boolean loggedValue = loggedNetworkBooleans.get(shortName).get();
            if (loggedValue != loggedBooleans.get(shortName)) {
                loggedBooleans.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkNumbers.keySet()) {
            double loggedValue = loggedNetworkNumbers.get(shortName).get();
            if (loggedValue != loggedNumbers.get(shortName)) {
                loggedNumbers.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }

        for (String shortName : loggedNetworkStrings.keySet()) {
            String loggedValue = loggedNetworkStrings.get(shortName).get();
            if (loggedValue != loggedStrings.get(shortName)) {
                loggedStrings.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }

        for (String shortName : loggedDashboardChoosers.keySet()) {
            Object loggedValue = loggedDashboardChoosers.get(shortName).get();
            if (!loggedValue.equals(loggedChoosers.get(shortName))) {
                loggedChoosers.put(shortName, loggedValue);
                hasChanged.put(shortName, true);
            }
        }
    }

    /**
     * Removes a value from the logger.
     * 
     * @param shortName The value to remove.
     */
    public static void remove(String shortName) {
        loggedNetworkBooleans.remove(shortName);
        loggedBooleans.remove(shortName);

        loggedNetworkNumbers.remove(shortName);
        loggedNumbers.remove(shortName);

        loggedNetworkStrings.remove(shortName);
        loggedStrings.remove(shortName);

        hasChanged.remove(shortName);
    }
}