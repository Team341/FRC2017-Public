package missdaisy.fileio;

import java.util.Hashtable;

/**
 * A class for representing name-value pairs of generic types.
 *
 * @author Jared341
 */
public class PropertySet {

  private static PropertySet mInstance = null;

  private final Hashtable<String, Object> mNameValueMap;

  /**
   * Get the instance of the PropertySet
   *
   * @return the instance of the PropertySet
   */
  public static PropertySet getInstance() {
    if (mInstance == null) {
      mInstance = new PropertySet();
    }
    return mInstance;
  }

  private PropertySet() {
    mNameValueMap = new Hashtable<String, Object>();
  }

  /**
   * Add a new name-value pair to the set. Overwrites any existing properties of the same name.
   *
   * @param aKey the key (name) of the property
   * @param aValue the value of the property
   */
  public void addProperty(String aKey, Object aValue) {
    try {
      mNameValueMap.put(aKey, aValue);
      System.out.println("Adding property Name: " + aKey + "; Value: " + aValue);
    } catch (NullPointerException e) {
    }
  }

  /**
   * Get the value associated with a key as a string.
   *
   * @param aKey the key
   * @param aDefault the default value to return in case of an error or inexistent property
   * @return the value represented as a string, or the default value if errors occur
   */
  public String getStringValue(String aKey, String aDefault) {
    if (mNameValueMap.containsKey(aKey)) {
      return String.valueOf(mNameValueMap.get(aKey));
    } else {
      return aDefault;
    }
  }

  /**
   * Get the value associated with a key as a double.
   *
   * @param aKey the key
   * @param aDefault the default value to return in case of an error or inexistent property
   * @return the value represented as a double, or the default value if errors occur
   */
  public double getDoubleValue(String aKey, double aDefault) {
    if (mNameValueMap.containsKey(aKey)) {
      double lRet = aDefault;

      try {
        lRet = Double.parseDouble(mNameValueMap.get(aKey).toString());
      } catch (NumberFormatException e) {
      }

      return lRet;
    } else {
      System.out.println("Could not find property: " + aKey);
      return aDefault;
    }
  }

  /**
   * Get the value associated with a key as an int.
   *
   * @param aKey the key
   * @param aDefault the default value to return in case of an error or inexistent property
   * @return the value represented as an int, or the default value if errors occur
   */
  public int getIntValue(String aKey, int aDefault) {
    if (mNameValueMap.containsKey(aKey)) {
      int lRet = aDefault;

      try {
        lRet = Integer.parseInt(mNameValueMap.get(aKey).toString());
      } catch (NumberFormatException e) {
      }

      return lRet;
    } else {
      return aDefault;
    }
  }

  public boolean hasProperty(String aKey) {
    return mNameValueMap.containsKey(aKey);
  }
}
