package org.unirail;

import org.xml.sax.Attributes;
import org.xml.sax.SAXException;
import org.xml.sax.helpers.DefaultHandler;

import javax.xml.parsers.SAXParser;
import javax.xml.parsers.SAXParserFactory;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashSet;


public class MavLink2AdHoc { //https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0
	
	
	static final String translate_datatype(String datatype, String len) {
		
		String attr = 0 < len.length() ? "[Dims( +" + len + " )] " : "";
		
		switch (datatype)
		{
			case "uint8_t":
				return attr + " byte ";
			case "uint8_t_mavlink_version":
			case "int8_t":
				return attr + " sbyte ";
			case "uint16_t":
				return attr + " ushort ";
			case "int16_t":
				return attr + " short ";
			case "uint32_t":
				return attr + " uint ";
			case "int32_t":
				return attr + " int ";
			case "uint64_t":
				return attr + " ulong ";
			case "int64_t":
				return attr + " long ";
			case "float":
				return attr + " float ";
			case "char":
				return attr + " char ";
			case "double":
				return attr + " double ";
			default:
				return null;
		}
	}
	
	static StringBuffer MAV_CMD = new StringBuffer();
	
	static class MyHandler extends DefaultHandler {
		
		public void parse(File file) {
			try
			{
				SAXParserFactory saxParserFactory = SAXParserFactory.newInstance();
				SAXParser        saxParser        = saxParserFactory.newSAXParser();
				saxParser.parse(file, this);
				
			} catch (Exception e) {}
		}
		
		
		private boolean is_BitFlags_enum = true;
		private boolean include          = false;
		private boolean long_enum        = false;
		
		@Override public void startElement(String uri, String localName, String qName, Attributes attributes) throws SAXException {
			super.startElement(uri, localName, qName, attributes);
			switch (qName)
			{
				case "include":
					include = true;
					break;
				case "message":
					sb1.setLength(0);
					sb2.setLength(0);
					sb3.setLength(0);
					//-   message: Each message is encapsulated by message tags, with the following
					//    attributes
					//
					//    -   id: The id attribute is the unique index number of this message (in the
					//        example above: 147).
					//
					//        -   For MAVLink 1:
					//
					//            -   Valid numbers range from 0 to 255.
					//
					//            -   The ids 0-149 and 230-255 are reserved for *common.xml*.
					//                Dialects can use 180-229 for custom messages (provided these are
					//                not used by other included dialects).
					//
					//        -   For [MAVLink 2](https://mavlink.io/en/guide/mavlink_2.html):
					//
					//            -   Valid numbers range from 0 to 16777215.
					//
					//            -   All numbers below 255 should be considered reserved unless
					//                messages are also intended for MAVLink 1.
					//
					//    -   name: The name attribute provides a human readable form for the message
					//        (ie "BATTERY_STATUS"). It is used for naming helper functions in
					//        generated libraries, but is not sent over the wire.
					//
					//-   description: Human readable description of message, shown in user interfaces
					//    and in code comments. This should contain all information (and hyperlinks)
					//    to fully understand the message.
					//
					
					sb1.append("class ").append(clean_name(attributes.getValue("name")));
					break;
				case "field":
					//-   field: Encodes one field of the message. The field value is its name/text
					//    string used in GUI documentation (but not sent over the wire). Every message
					//    must have at least one field.
					//
					//    -   type: Similar to a field in a C struct - the size of the data required
					//        to store/represent the data type.
					//
					//        -   Fields can be signed/unsigned integers of size 8, 16, 23, 64 bits
					//            ({u)int8_t, (u)int16_t, (u)int32_t, (u)int64_t), single/double
					//            precision IEEE754 floating point numbers. They can also be arrays of
					//            the other types - e.g. uint16_t[10].
					//
					//    -   name: Name of the field (used in code).
					//
					//    -   [enum](https://mavlink.io/en/guide/xml_schema.html#enum) (optional):
					//        Name of an enum defining possible values of the field
					//        (e.g. MAV_BATTERY_CHARGE_STATE).
					//
					//    -   units (optional): The units for message fields that take numeric values
					//        (not enums). These are defined in
					//        the [schema](https://github.com/ArduPilot/pymavlink/blob/master/generator/mavschema.xsd) (search
					//        on *name="SI_Unit"*)
					//
					//    -   display (optional): This should be set as display="bitmask" for bitmask
					//        fields (hint to ground station that enum values must be displayed as
					//        checkboxes).
					//
					//    -   print_format (optional): TBD.
					//
					//    -   default (optional): TBD.
					//
					//    -   instance: If true, this indicates that the message contains the
					//        information for a particular sensor or battery (e.g. Battery 1, Battery
					//        2, etc.) and that this field indicates which sensor. Default is false.
					//
					//>   This field allows a recipient automatically associate messages for a
					//>   particular sensor and plot them in the same series.
					//
					//-   invalid: Specifies a value that can be set on a field to indicate that the
					//    data is *invalid*: the recipient should ignore the field if it has this
					//    value. For example, BATTERY_STATUS.current_battery specifies invalid="-1",
					//    so a battery that does not measure supplied *current* should
					//    set BATTERY_STATUS.current_battery to -1.
					//
					//>   Where possible the value that indicates the field is invalid should be
					//>   selected to outside the expected/valid range of the field (0 is preferred if
					//>   it is not an acceptable value for the field). For integers we usually select
					//>   the largest possible value
					//>   (i.e. UINT16_MAX, INT16_MAX, UINT8_MAX, UINT8_MAX). For floats we usually
					//>   select invalid="NaN".
					//
					//>   Arrays represent multiple elements, some (or all) of which may need to be
					//>   marked as invalid. The following notation is used to specify the values that
					//>   indicate elements of the array are invalid:
					//
					//-   invalid="[value]": Array elements that contain value are invalid.
					//
					//-   invalid="[value:]": All array elements are invalid if the *first* array
					//    element is set to value.
					//
					//-   invalid="[value1,,value3,]": Array elements are invalid if they contain the
					//    value specified in the corresponding position of the comma separated list.
					//    If the a position in the list is empty, there is no way to indicate the
					//    corresponding array element is invalid. The example above indicates that
					//    elements 1 and 3 are invalid if they contain value1 and value3,
					//    respectively. For element 2 and any elements \>4 the invalid property of the
					//    field cannot be set.
					//
					//-   invalid="[value1,]": The first array element is invalid if it
					//    contains value1: the invalid property cannot be set for all other elements.
					
					String datatype = attributes.getValue("type");
					
					int pos = datatype.indexOf("[");
					if (0 < pos)
					{
						String len = datatype.substring(pos + 1).replace("]", "");
						
						datatype = datatype.contains("char") ? "string " : translate_datatype(datatype.substring(0, pos), len);
					}
					else
						datatype = translate_datatype(datatype, "");
					
					
					String e = attributes.getValue("enum");
					sb3.append(e == null ? datatype : e).append(" ").append(clean_name(attributes.getValue("name"))).append(";");
					break;
				case "enum":
					//The main enum tags/fields are:
					//
					//-   name: The name of the enum (mandatory). This is a string of capitalized,  underscore-separated words.
					//
					//-   description (optional): A string describing the purpose of the enum
					//
					//-   entry (optional): An entry (zero or more entries can be specified for each enum)
					//
					//-   [deprecated](https://mavlink.io/en/guide/xml_schema.html#deprecated) (optional):   A tag indicating that the enum is deprecated.
					
					sb1.setLength(0);
					sb2.setLength(0);
					sb3.setLength(0);
					
					long_enum = false;
					is_BitFlags_enum = true;
					
					String enum_name = clean_name(attributes.getValue("name"));
					
					is_MAV_CMD = enum_name.equals("MAV_CMD");
					
					sb1.append("enum ").append(enum_name);
					
					break;
				case "entry":
					
					//The "normal" enum entry tags/fields are:
					//
					//-   name: The name of the enum value (mandatory). This is a string of
					//    capitalized, underscore-separated words.
					//
					//-   value (optional): The *value* for the entry (a number).
					//
					//-   description (optional): A description of the entry.
					//
					//-   [deprecated](https://mavlink.io/en/guide/xml_schema.html#deprecated) / [wip](https://mavlink.io/en/guide/xml_schema.html#wip) (optional):
					//    A tag indicating that the enum is deprecated or "work in progress".
					
					
					String value = attributes.getValue("value");
					String name = clean_name(attributes.getValue("name"));
					
					if (value != null)
					{
						if (is_BitFlags_enum)
							try//            detect Flags enums
							{
								long val = Long.decode(value);
								if (!long_enum && val < Integer.MIN_VALUE || Integer.MAX_VALUE < val) long_enum = true;
								if (0 < val) is_BitFlags_enum = (val & (val - 1)) == 0;
								
							} catch (NumberFormatException e1) {}
						
						sb3.append(name).append(" = ").append(value).append(", ");
					}
					else
						sb3.append(clean_name(attributes.getValue("name"))).append(", ");
					
					if (is_MAV_CMD)
					{
						entry.setLength(0);
						entry.append("interface ").append(name).append("{\n");
					}
					break;
				
				case "param":
					
					//A param **must** include the following attribute:
					//
					//-   index - The parameter number (1 - 7).
					//
					//A param **should** have:
					//
					//-   description: Parameter description string (tag body)
					//
					//A param **should** also include the following optional attributes where
					//appropriate (which may be used by a GUI for parameter display and editing):
					//
					//-   label - Display name to represent the parameter in a GCS or other UI. All
					//    words in label should be capitalised.
					//
					//-   units - SI units for the value.
					//
					//-   enum - Enum containing possible values for the parameter (if applicable).
					//
					//-   decimalPlaces - Hint to a UI about how many decimal places to use if the
					//    parameter value is displayed.
					//
					//-   increment - Allowed increments for the parameter value.
					//
					//-   minValue - Minimum value for param.
					//
					//-   maxValue - Maximum value for the param.
					//
					//-   reserved - Boolean indicating whether param is reserved for future use. If
					//    the attributes is
					//
					//-   default - Default value for the param (primarily used for reserved params,
					//    where the value is 0 or NaN).
					
					
					param.append("public interface param_").append(attributes.getValue("index")).append("{\n");
					String label = attributes.getValue("label");
					if (label != null) param.append("public const string label = \"").append(label).append("\";\n");
					String Enum = attributes.getValue("enum");
					if (Enum != null) param.append("public const string Enum = \"").append(Enum).append("\";\n");
					String decimalPlaces = attributes.getValue("decimalPlaces");
					if (decimalPlaces != null) param.append("public const string decimalPlaces = \"").append(decimalPlaces).append("\";\n");
					String increment = attributes.getValue("increment");
					if (increment != null) param.append("public const string increment = \"").append(increment).append("\";\n");
					String minValue = attributes.getValue("minValue");
					if (minValue != null) param.append("public const string minValue = \"").append(minValue).append("\";\n");
					String maxValue = attributes.getValue("maxValue");
					if (maxValue != null) param.append("public const string maxValue = \"").append(maxValue).append("\";\n");
					String reserved = attributes.getValue("reserved");
					if (reserved != null) param.append("public const string reserved = \"").append(reserved).append("\";\n");
					String Default = attributes.getValue("default");
					if (Default != null) param.append("public const string Default = \"").append(Default).append("\";\n");
					
					
					break;
				
				case "description":
					
					break;
			}
		}
		static final StringBuilder sb1   = new StringBuilder();
		static final StringBuilder sb2   = new StringBuilder();
		static final StringBuilder sb3   = new StringBuilder();
		static final StringBuilder entry = new StringBuilder();
		static final StringBuilder param = new StringBuilder();
		
		static final StringBuilder text1 = new StringBuilder();
		static final StringBuilder text2 = new StringBuilder();
		static final StringBuilder text3 = new StringBuilder();
		
		static void doc(StringBuilder src, StringBuilder dst) {
			dst.append("\n/**\n").append(src).append("\n*/\n");
			src.setLength(0);
		}
		@Override public void characters(char[] chars, int start, int length) throws SAXException {
			
			super.characters(chars, start, length);
			while ((chars[start] < 33) && 0 < length)//skip none printed symbol(garbage)
			{
				start++;
				length--;
			}
			if (length < 1) return;
			
			StringBuilder text = 0 < param.length() ? text3 : 0 < sb3.length() ? text2 : text1;
			text.setLength(0);
			
			if (include)
			{
				text.append(chars, start, length);
				return;
			}
			
			for (int i = 0, len = 0; i < length; i++, len++)
			{
				char ch = chars[start + i];
				if (ch == ' ' && max_width < len)
				{
					text.append('\n');
					len = 0;
				}
				else text.append(ch);
			}
		}
		
		@Override public void endElement(String uri, String localName, String qName) throws SAXException {
			super.endElement(uri, localName, qName);
			switch (qName)
			{
				
				case "include"://include other xml descriptor
					String file = text1.toString();
					if (file_names.add(file))
					{
						final MyHandler handler = new MyHandler();
						handler.parse(Paths.get(path, file).toFile());
					}
					include = false;
					break;
				
				case "enum":
					if (is_MAV_CMD)
					{
						is_MAV_CMD = false;
						
						if (0 < text1.length() && MAV_CMD_description.length() == 0)
						{
							text1.setLength(0);
							MAV_CMD_description = text1.toString();
						}
						
						MAV_CMD.append(sb2);
						
						sb1.setLength(0);
						sb2.setLength(0);
						
						break;
					}
					if (long_enum) sb1.append(":long");
				case "message":
					if (0 < text1.length()) doc(text1, packs);
					packs.append(sb1).append("{\n");
					packs.append(sb2);
					packs.append("\n}\n");
					
					sb1.setLength(0);
					sb2.setLength(0);
					break;
				case "param":
					entry.append(param);
					param.setLength(0);
					
					if (0 < text3.length())
					{
						entry.append(" public const string description = @\"").append(text3.toString().replace("\"", "\"\"")).append("\";\n");
						text3.setLength(0);
					}
					entry.append("\n}\n");
					break;
				
				case "entry":
					
					if (is_MAV_CMD)
					{
						//root
						//  entry
						//      param
						root.append(entry).append('\n');
						entry.setLength(0);
						
						if (0 < text2.length()) root.append(" public const string description = @\"").append(text2.toString().replace("\"", "\"\"")).append("\";\n");
						
						root.append("\n}\n");
					}
				
				case "field":
					
					if (0 < text2.length()) doc(text2, sb2);
					sb2.append(sb3).append('\n');
					
					sb3.setLength(0);
					break;
			}
		}
	}
	
	static String path = "";
	
	static final HashSet<String> file_names          = new HashSet<>();
	static       StringBuilder   packs               = new StringBuilder();
	static       StringBuilder   root                = new StringBuilder();
	static       String          MAV_CMD_description = "";
	static       boolean         is_MAV_CMD          = false;
	
	static final int max_width = 100;
	
	// command argument: path to the folder with MavLink XML description files
	public static void main(String[] args) {
		
		final MyHandler handler = new MyHandler();
		File            dir     = new File(path = args[0]);
		File[]          files   = dir.listFiles((dir1, name) -> name.endsWith(".xml"));
		if (files == null)
		{
			System.out.println("No files in the folder `" + dir + "` found.");
			try
			{
				System.in.read();
			} catch (IOException ignored) {}
			return;
		}
		
		try
		{
			for (File file : files)
			{
				file_names.clear();
				
				
				handler.parse(file);
				
				Files.write(Paths.get(file.getPath().replace(".xml", ".cs")),
				            (
						            "using System;\nusing org.unirail.Meta;\n namespace org.mavlink {\n" +
						            "public interface " + file.getName().replace(".xml", "") + "{\n " +
						            "interface CommunicationChannel : GroundControl.CommunicationInterface, MicroAirVehicle.CommunicationInterface {}\n" +
						            "class GroundControl :  InJAVA, InCS{\n"
						            + "     public interface CommunicationInterface  { "
						            + packs
						            + (0 < MAV_CMD.length() ? MAV_CMD_description + "enum MAV_CMD {\n" + MAV_CMD + "\n}\n" : "")
						            + (0 < root.length() ? "interface MAV_CMD_PARAMS {\n" + root + "\n}\n" : "")
						            + SI_Unit
						            + "}\n}\n"
						            + "class MicroAirVehicle :  InCS, InCPP{\n"
						            + "     public interface CommunicationInterface  {}\n"
						            + "}\n" +
						            "\n}\n}\n"
				            ).getBytes(StandardCharsets.UTF_8));
				
				MAV_CMD.setLength(0);
				root.setLength(0);
				packs.setLength(0);
				MAV_CMD_description = "";
				is_MAV_CMD          = false;
				MyHandler.sb1.setLength(0);
				MyHandler.sb2.setLength(0);
				MyHandler.sb3.setLength(0);
				MyHandler.entry.setLength(0);
				MyHandler.param.setLength(0);
				MyHandler.text1.setLength(0);
				MyHandler.text2.setLength(0);
				MyHandler.text3.setLength(0);
			}
		} catch (Exception e)
		{
			e.printStackTrace();
		}
	}
	
	static String clean_name(String name) {
		if (!is_prohibited(name)) return name;
		
		String new_name = name;
		
		for (int i = 0; i < name.length(); i++)
		{
			char ch = name.charAt(i);
			if (Character.isLowerCase(ch))
			{
				new_name = new_name.substring(0, i) + Character.toUpperCase(ch) + new_name.substring((i + 1), new_name.length() - 1);
				if (is_prohibited(new_name)) continue;
				return new_name;
			}
		}
		
		return name;
	}
	static boolean is_prohibited(String name) {
		switch (name)
		{
			case "abstract":
			case "assert":
			case "become":
			case "bool":
			case "boolean":
			case "box":
			case "break":
			case "by":
			case "byte":
			case "case":
			case "cast":
			case "catch":
			case "char":
			case "char16_t":
			case "char32_t":
			case "checked":
			case "class":
			case "companion":
			case "const":
			case "const_cast":
			case "constexpr":
			case "constructor":
			case "continue":
			case "crate":
			case "crossinline":
			case "data":
			case "debugger":
			case "decimal":
			case "declare":
			case "decltype":
			case "default":
			case "delegate":
			case "delete":
			case "deprecated":
			case "dllexport":
			case "dllimport":
			case "do":
			case "double":
			case "dst":
			case "dyn":
			case "dynamic":
			case "dynamic_cast":
			case "each":
			case "else":
			case "enum":
			case "Error":
			case "eval":
			case "event":
			case "expect":
			case "explicit":
			case "export":
			case "extends":
			case "extern":
			case "external":
			case "false":
			case "field":
			case "file":
			case "final":
			case "finally":
			case "fixed":
			case "float":
			case "fn":
			case "for":
			case "foreach":
			case "friend":
			case "from":
			case "fun":
			case "function":
			case "gcnew":
			case "generic":
			case "get":
			case "goto":
			case "i128":
			case "i16":
			case "i32":
			case "i64":
			case "i8":
			case "if":
			case "impl":
			case "implements":
			case "implicit":
			case "import":
			case "in":
			case "infix":
			case "init":
			case "inline":
			case "inner":
			case "instanceof":
			case "int":
			case "int16_t":
			case "int32_t":
			case "int64_t":
			case "int8_t":
			case "interface":
			case "interior":
			case "internal":
			case "is":
			case "lateinit":
			case "let":
			case "literal":
			case "lock":
			case "long":
			case "loop":
			case "macro":
			case "match":
			case "mod":
			case "module":
			case "move":
			case "mut":
			case "mutable":
			case "naked":
			case "namespace":
			case "native":
			case "new":
			case "noexcept":
			case "noinline":
			case "noreturn":
			case "nothrow":
			case "novtable":
			case "null":
			case "nullptr":
			case "number":
			case "object":
			case "only":
			case "open":
			case "operator":
			case "out":
			case "override":
			case "pack":
			case "package":
			case "param":
			case "params":
			case "priv":
			case "private":
			case "property":
			case "protected":
			case "ptr":
			case "pub":
			case "public":
			case "readonly":
			case "receiver":
			case "ref":
			case "register":
			case "reified":
			case "reinterpret_":
			case "reinterpret_cast":
			case "require":
			case "return":
			case "safecast":
			case "sbyte":
			case "sealed":
			case "selectany":
			case "Self":
			case "set":
			case "setparam":
			case "short":
			case "signed":
			case "sizeof":
			case "src":
			case "stackalloc":
			case "static":
			case "static_assert":
			case "static_cast":
			case "str":
			case "strictfp":
			case "string":
			case "struct":
			case "super":
			case "suspend":
			case "switch":
			case "symbol":
			case "synchronized":
			case "tailrec":
			case "template":
			case "this":
			case "thread":
			case "throw":
			case "throws":
			case "trait":
			case "transient":
			case "true":
			case "try":
			case "type":
			case "typealias":
			case "typedef":
			case "typeid":
			case "typename":
			case "typeof":
			case "u128":
			case "u16":
			case "u32":
			case "u64":
			case "u8":
			case "uint":
			case "uint16_t":
			case "uint32_t":
			case "uint64_t":
			case "ulong":
			case "unchecked":
			case "union":
			case "unsafe":
			case "unsigned":
			case "unsized":
			case "use":
			case "ushort":
			case "using":
			case "uuid":
			case "val":
			case "value":
			case "vararg":
			case "virtual":
			case "void":
			case "volatile":
			case "wchar_t":
			case "where":
			case "while":
			case "with":
			case "yield":
				
				
				return true;
		}
		
		return false;
	}
	
	static final String SI_Unit = "interface SI_Unit\n" +
	                              "    {\n" +
	                              "        interface time\n" +
	                              "        {\n" +
	                              "            const string s   = \"s\";   // seconds\n" +
	                              "            const string ds  = \"ds\";  // deciseconds\n" +
	                              "            const string cs  = \"cs\";  // centiseconds\n" +
	                              "            const string ms  = \"ms\";  // milliseconds\n" +
	                              "            const string us  = \"us\";  // microseconds\n" +
	                              "            const string Hz  = \"Hz\";  // Herz\n" +
	                              "            const string MHz = \"MHz\"; // Mega-Herz\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface distance\n" +
	                              "        {\n" +
	                              "            const string km    = \"km\";    // kilometres\n" +
	                              "            const string dam   = \"dam\";   // decametres\n" +
	                              "            const string m     = \"m\";     // metres\n" +
	                              "            const string m_s   = \"m/s\";   // metres per second\n" +
	                              "            const string m_s_s = \"m/s/s\"; // metres per second per second\n" +
	                              "            const string m_s_5 = \"m/s*5\"; // metres per second * 5 required from dagar for HIGH_LATENCY2 message\n" +
	                              "            const string dm    = \"dm\";    // decimetres\n" +
	                              "            const string dm_s  = \"dm/s\";  // decimetres per second\n" +
	                              "            const string cm    = \"cm\";    // centimetres\n" +
	                              "            const string cm_2  = \"cm^2\";  // centimetres squared (typically used in variance)\n" +
	                              "            const string cm_s  = \"cm/s\";  // centimetres per second\n" +
	                              "            const string mm    = \"mm\";    // millimetres\n" +
	                              "            const string mm_s  = \"mm/s\";  // millimetres per second\n" +
	                              "            const string mm_h  = \"mm/h\";  // millimetres per hour\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface temperature\n" +
	                              "        {\n" +
	                              "            const string K     = \"K\";     // Kelvin\n" +
	                              "            const string degC  = \"degC\";  // degrees Celsius\n" +
	                              "            const string cdegC = \"cdegC\"; // centi degrees Celsius\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface angle\n" +
	                              "        {\n" +
	                              "            const string rad    = \"rad\";    // radians\n" +
	                              "            const string rad_s  = \"rad/s\";  // radians per second\n" +
	                              "            const string mrad_s = \"mrad/s\"; // milli-radians per second\n" +
	                              "            const string deg    = \"deg\";    // degrees\n" +
	                              "            const string deg_2  = \"deg/2\";  // degrees/2 required from dagar for HIGH_LATENCY2 message\n" +
	                              "            const string deg_s  = \"deg/s\";  // degrees per second\n" +
	                              "            const string cdeg   = \"cdeg\";   // centidegrees\n" +
	                              "            const string cdeg_s = \"cdeg/s\"; // centidegrees per second\n" +
	                              "            const string degE5  = \"degE5\";  // degrees * 10E5\n" +
	                              "            const string degE7  = \"degE7\";  // degrees * 10E7\n" +
	                              "            const string rpm    = \"rpm\";    // rotations per minute\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface electricity\n" +
	                              "        {\n" +
	                              "            const string V   = \"V\";   // Volt\n" +
	                              "            const string cV  = \"cV\";  // centi-Volt\n" +
	                              "            const string mV  = \"mV\";  // milli-Volt\n" +
	                              "            const string A   = \"A\";   // Ampere\n" +
	                              "            const string cA  = \"cA\";  // centi-Ampere\n" +
	                              "            const string mA  = \"mA\";  // milli-Ampere\n" +
	                              "            const string mAh = \"mAh\"; // milli-Ampere hour\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface magnetism\n" +
	                              "        {\n" +
	                              "            const string mT     = \"mT\";     // milli-Tesla\n" +
	                              "            const string gauss  = \"gauss\";  // Gauss\n" +
	                              "            const string mgauss = \"mgauss\"; // milli-Gauss\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface energy\n" +
	                              "        {\n" +
	                              "            const string hJ = \"hJ\"; // hecto-Joule\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface power\n" +
	                              "        {\n" +
	                              "            const string W = \"W\"; // Watt\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface force\n" +
	                              "        {\n" +
	                              "            const string mG = \"mG\"; // milli-G\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface mass\n" +
	                              "        {\n" +
	                              "            const string g  = \"g\";  // grams\n" +
	                              "            const string kg = \"kg\"; // kilograms\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface pressure\n" +
	                              "        {\n" +
	                              "            const string Pa   = \"Pa\";   // Pascal\n" +
	                              "            const string hPa  = \"hPa\";  // hecto-Pascal\n" +
	                              "            const string kPa  = \"kPa\";  // kilo-Pascal\n" +
	                              "            const string mbar = \"mbar\"; // millibar\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface ratio\n" +
	                              "        {\n" +
	                              "            const string percent      = \"%\";   // percent\n" +
	                              "            const string decipercent  = \"d%\";  // decipercent\n" +
	                              "            const string centipercent = \"c%\";  // centipercent\n" +
	                              "            const string dB           = \"dB\";  // Deci-Bell\n" +
	                              "            const string dBm          = \"dBm\"; // Deci-Bell-milliwatts\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface digital\n" +
	                              "        {\n" +
	                              "            const string KiB     = \"KiB\";     // Kibibyte (1024 bytes)\n" +
	                              "            const string KiB_s   = \"KiB/s\";   // Kibibyte (1024 bytes) per second\n" +
	                              "            const string MiB     = \"MiB\";     // Mebibyte (1024*1024 bytes)\n" +
	                              "            const string MiB_s   = \"MiB/s\";   // Mebibyte (1024*1024 bytes) per second\n" +
	                              "            const string bytes   = \"bytes\";   // bytes\n" +
	                              "            const string bytes_s = \"bytes/s\"; // bytes per second\n" +
	                              "            const string bits_s  = \"bits/s\";  // bits per second\n" +
	                              "            const string pix     = \"pix\";     // pixels\n" +
	                              "            const string dpix    = \"dpix\";    // decipixels\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface flow\n" +
	                              "        {\n" +
	                              "            const string g_min    = \"g/min\";    // grams/minute\n" +
	                              "            const string cm_3_min = \"cm^3/min\"; // cubic centimetres/minute\n" +
	                              "        }\n" +
	                              "\n" +
	                              "        interface volume\n" +
	                              "        {\n" +
	                              "            const string cm_3 = \"cm^3\"; // cubic centimetres\n" +
	                              "        }\n" +
	                              "    }";
}

