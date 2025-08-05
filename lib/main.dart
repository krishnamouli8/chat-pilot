import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;
import 'package:flutter_tts/flutter_tts.dart';
import 'package:google_generative_ai/google_generative_ai.dart';
import 'package:permission_handler/permission_handler.dart';
import 'dart:convert';

// Replace with your actual Gemini API key
const String GEMINI_API_KEY = 'AIzaSyCoGn4AtSE1PLiOQU2Iia10N2Nb8dDh39M';

void main() {
  runApp(const RoverControlApp());
}

class RoverControlApp extends StatelessWidget {
  const RoverControlApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Rover Control',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.light(
          primary: const Color(0xFF0D47A1), // A deeper blue for a more technical feel
          secondary: const Color(0xFFF9A825), // A gold/yellow for accents
          surface: Colors.white,
          background: const Color(0xFFF8FAFC),
        ),
        fontFamily: 'SF Pro Display',
      ),
      debugShowCheckedModeBanner: false,
      home: const ChatScreen(),
    );
  }
}

class ChatScreen extends StatefulWidget {
  const ChatScreen({super.key});

  @override
  State<ChatScreen> createState() => _ChatScreenState();
}

class _ChatScreenState extends State<ChatScreen> with TickerProviderStateMixin {
  final ScrollController _scrollController = ScrollController();
  final TextEditingController _textController = TextEditingController();
  final FocusNode _focusNode = FocusNode();

  bool isListening = false;
  bool isProcessing = false;
  bool isSpeaking = false;
  bool _isTypingMode = false;

  String _currentSpeechText = '';
  final List<ChatMessage> _messages = [];

  late stt.SpeechToText _speech;
  late FlutterTts _flutterTts;
  late GenerativeModel _model;

  late AnimationController _pulseController;
  late Animation<double> _pulseAnimation;

  @override
  void initState() {
    super.initState();
    _initializeServices();
    _setupAnimations();
  }

  void _initializeServices() {
    _speech = stt.SpeechToText();
    _initializeTts();
    _initializeGemini();
    _requestPermissions();
  }

  void _setupAnimations() {
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    )..repeat(reverse: true);

    _pulseAnimation = Tween<double>(
      begin: 0.8,
      end: 1.0,
    ).animate(CurvedAnimation(
      parent: _pulseController,
      curve: Curves.easeInOut,
    ));
  }

  void _initializeTts() {
    _flutterTts = FlutterTts();
    _flutterTts.setStartHandler(() => setState(() => isSpeaking = true));
    _flutterTts.setCompletionHandler(() => setState(() => isSpeaking = false));
    _flutterTts.setErrorHandler((msg) => setState(() => isSpeaking = false));
    _flutterTts.setSpeechRate(0.5);
    _flutterTts.setVolume(1.0);
    _flutterTts.setPitch(1.0);
  }

  void _initializeGemini() {
    _model = GenerativeModel(
      model: 'gemini-1.5-flash',
      apiKey: GEMINI_API_KEY,
    );
  }

  Future<void> _requestPermissions() async {
    await Permission.microphone.request();
  }

  @override
  void dispose() {
    _pulseController.dispose();
    _scrollController.dispose();
    _textController.dispose();
    _focusNode.dispose();
    _flutterTts.stop();
    super.dispose();
  }

  Future<void> _startListening() async {
    if (isProcessing) return;

    bool available = await _speech.initialize(
      onStatus: (status) {
        if (status == 'done' || status == 'notListening') {
          if (_currentSpeechText.isNotEmpty) {
            _stopListening();
          }
        }
      },
      onError: (error) => _stopListening(),
    );

    if (available) {
      setState(() {
        isListening = true;
        _currentSpeechText = '';
      });

      _speech.listen(
        onResult: (result) => setState(() => _currentSpeechText = result.recognizedWords),
        listenFor: const Duration(seconds: 30),
        pauseFor: const Duration(seconds: 3),
        partialResults: true,
      );
    }
  }

  Future<void> _stopListening() async {
    await _speech.stop();
    setState(() => isListening = false);

    if (_currentSpeechText.isNotEmpty) {
      _sendCommand(_currentSpeechText);
      _currentSpeechText = '';
    }
  }

  void _toggleListening() {
    isListening ? _stopListening() : _startListening();
    HapticFeedback.lightImpact();
  }

  Future<void> _sendCommand(String text) async {
    if (text.trim().isEmpty || isProcessing) return;

    _textController.clear();
    final userMessage = ChatMessage(
      text: text.trim(),
      isUser: true,
      timestamp: DateTime.now(),
    );

    setState(() {
      _messages.add(userMessage);
      isProcessing = true;
    });
    _scrollToBottom();

    final assistantMessage = ChatMessage(
      text: '',
      isUser: false,
      timestamp: DateTime.now(),
      isProcessing: true,
    );
    setState(() => _messages.add(assistantMessage));
    _scrollToBottom();

    try {
      // Build the chat history string to provide context to the model.
      final history = _messages.length > 1
          ? _messages.sublist(0, _messages.length - 1)
          : [];
      final historyString = history
          .map((m) => "${m.isUser ? 'User' : 'Assistant'}: ${m.text}")
          .join('\n');

      // The new prompt instructs the model to be conversational and choose a response type.
      final prompt = '''
        You are a conversational AI assistant for an autonomous rover. Your goal is to be helpful and conversational while also being able to execute commands.
        Analyze the user's input based on the conversation history. You have two possible response types: 'command' or 'conversation'.

        1.  If the input is a direct command for the rover, respond with a JSON object where "responseType" is "command". This JSON should also contain a "command" and "parameters" field.
            - Valid commands: "move", "turn", "stop", "status", "scan".
            - Example command input: "Move forward 5 meters"
            - Example command output: {"responseType": "command", "command": "move", "parameters": {"direction": "forward", "distance": "5 meters"}}

        2.  If the input is a question, a follow-up, or a general statement, respond with a JSON object where "responseType" is "conversation". This JSON should also contain a "responseText" field with a natural, helpful, and conversational answer.
            - Example question input: "Why did you stop?"
            - Example conversation output: {"responseType": "conversation", "responseText": "I stopped because my sensors detected an obstacle ahead."}

        Here is the recent conversation history for context:
        $historyString

        Here is the current user input:
        User: "$text"

        Respond ONLY with a single, valid JSON object.
      ''';

      final content = [Content.text(prompt)];
      final response = await _model.generateContent(content);
      
      String rawResponse = response.text ?? '{"responseType": "conversation", "responseText": "Sorry, I had trouble understanding."}';
      rawResponse = rawResponse.replaceAll('```json', '').replaceAll('```', '').trim();

      final Map<String, dynamic> responseJson = jsonDecode(rawResponse);
      final String responseType = responseJson['responseType'] ?? 'conversation';
      
      String responseText;

      if (responseType == 'command') {
        final String command = responseJson['command'] ?? 'unknown';
        final Map<String, dynamic> parameters = responseJson['parameters'] ?? {};

        if (command != 'unknown' && command != 'error') {
          responseText = "Command received: ${command.toUpperCase()}.";
          if (parameters.isNotEmpty) {
            responseText += " Parameters: ${parameters.toString()}";
          }
          
          // =================================================================
          // TODO: SEND THE PARSED COMMAND TO THE ROVER'S CONTROL SYSTEM
          // Example: sendToRover(command, parameters);
          // =================================================================
          print("Executing command: $command with parameters: $parameters");

        } else {
          responseText = "Sorry, I didn't understand that command. Please try again.";
        }
      } else { // 'conversation'
        responseText = responseJson['responseText'] ?? "I'm not sure how to respond to that.";
      }

      setState(() {
        _messages.last = ChatMessage(
          text: responseText,
          isUser: false,
          timestamp: DateTime.now(),
        );
        isProcessing = false;
      });

      _scrollToBottom();
      await _flutterTts.speak(responseText);

    } catch (e) {
      print('Error processing command: $e');
      final errorText = 'Sorry, an error occurred while interpreting your command.';
      setState(() {
        _messages.last = ChatMessage(
          text: errorText,
          isUser: false,
          timestamp: DateTime.now(),
        );
        isProcessing = false;
      });
      _scrollToBottom();
      await _flutterTts.speak(errorText);
    }
  }

  void _scrollToBottom() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_scrollController.hasClients) {
        _scrollController.animateTo(
          _scrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 300),
          curve: Curves.easeOut,
        );
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Theme.of(context).colorScheme.background,
      appBar: AppBar(
        backgroundColor: Colors.white,
        elevation: 1,
        shadowColor: Colors.black.withOpacity(0.1),
        title: const Text(
          'Rover Voice Command',
          style: TextStyle(
            color: Color(0xFF1E293B),
            fontSize: 18,
            fontWeight: FontWeight.w600,
          ),
        ),
        centerTitle: true,
        actions: [
          IconButton(
            icon: const Icon(Icons.settings_outlined, color: Color(0xFF64748B)),
            onPressed: _showSettings,
          ),
        ],
      ),
      body: Column(
        children: [
          Expanded(
            child: _messages.isEmpty ? _buildEmptyState() : _buildMessageList(),
          ),
          _buildInputArea(),
        ],
      ),
    );
  }

  Widget _buildEmptyState() {
    return Center(
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Container(
            width: 80,
            height: 80,
            decoration: BoxDecoration(
              color: Theme.of(context).colorScheme.primary.withOpacity(0.1),
              borderRadius: BorderRadius.circular(40),
            ),
            child: Icon(
              Icons.smart_toy_outlined,
              size: 40,
              color: Theme.of(context).colorScheme.primary,
            ),
          ),
          const SizedBox(height: 24),
          const Text(
            'Rover is Ready',
            style: TextStyle(
              fontSize: 22,
              fontWeight: FontWeight.w600,
              color: Color(0xFF1E293B),
            ),
          ),
          const SizedBox(height: 8),
          const Text(
            'Tap the microphone to issue a command',
            style: TextStyle(
              fontSize: 16,
              color: Color(0xFF64748B),
            ),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }

  Widget _buildMessageList() {
    return ListView.builder(
      controller: _scrollController,
      padding: const EdgeInsets.all(16),
      itemCount: _messages.length,
      itemBuilder: (context, index) {
        final message = _messages[index];
        return _buildMessageBubble(message);
      },
    );
  }

  Widget _buildMessageBubble(ChatMessage message) {
    final isUser = message.isUser;
    final theme = Theme.of(context);
    
    return Container(
      margin: const EdgeInsets.only(bottom: 16),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        mainAxisAlignment: isUser ? MainAxisAlignment.end : MainAxisAlignment.start,
        children: [
          if (!isUser) ...[
            Container(
              width: 32,
              height: 32,
              decoration: BoxDecoration(
                color: theme.colorScheme.primary,
                borderRadius: BorderRadius.circular(16),
              ),
              child: const Icon(Icons.smart_toy, color: Colors.white, size: 18),
            ),
            const SizedBox(width: 12),
          ],
          Flexible(
            child: Column(
              crossAxisAlignment: isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
              children: [
                Container(
                  padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                  decoration: BoxDecoration(
                    color: isUser ? theme.colorScheme.primary : Colors.white,
                    borderRadius: BorderRadius.circular(18).copyWith(
                      bottomLeft: isUser ? const Radius.circular(18) : const Radius.circular(4),
                      bottomRight: isUser ? const Radius.circular(4) : const Radius.circular(18),
                    ),
                    boxShadow: [
                      BoxShadow(
                        color: Colors.black.withOpacity(0.05),
                        blurRadius: 10,
                        offset: const Offset(0, 2),
                      ),
                    ],
                  ),
                  child: message.isProcessing ? _buildProcessingIndicator() : Text(
                    message.text,
                    style: TextStyle(
                      color: isUser ? Colors.white : const Color(0xFF1E293B),
                      fontSize: 16,
                      height: 1.4,
                    ),
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  _formatTime(message.timestamp),
                  style: const TextStyle(color: Color(0xFF94A3B8), fontSize: 12),
                ),
              ],
            ),
          ),
          if (isUser) ...[
            const SizedBox(width: 12),
            Container(
              width: 32,
              height: 32,
              decoration: BoxDecoration(
                color: const Color(0xFF64748B),
                borderRadius: BorderRadius.circular(16),
              ),
              child: const Icon(Icons.person, color: Colors.white, size: 18),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildProcessingIndicator() {
    return Row(
      mainAxisSize: MainAxisSize.min,
      children: [
        SizedBox(
          width: 16,
          height: 16,
          child: CircularProgressIndicator(
            strokeWidth: 2,
            valueColor: AlwaysStoppedAnimation<Color>(const Color(0xFF64748B).withOpacity(0.6)),
          ),
        ),
        const SizedBox(width: 8),
        const Text(
          'Interpreting command...',
          style: TextStyle(
            color: Color(0xFF64748B),
            fontSize: 16,
            fontStyle: FontStyle.italic,
          ),
        ),
      ],
    );
  }

  Widget _buildInputArea() {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            blurRadius: 10,
            offset: const Offset(0, -2),
          ),
        ],
      ),
      child: SafeArea(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            if (isListening) _buildListeningIndicator(),
            Row(
              children: [
                _buildModeToggleButton(),
                const SizedBox(width: 8),
                Expanded(
                  child: _isTypingMode ? _buildTextField() : _buildMicButton(),
                ),
                const SizedBox(width: 8),
                _isTypingMode ? _buildSendButton() : const SizedBox(width: 48),
              ],
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildModeToggleButton() {
    return GestureDetector(
      onTap: () {
        if (isListening) _stopListening();
        setState(() {
          _isTypingMode = !_isTypingMode;
          if (_isTypingMode) {
            _focusNode.requestFocus();
          } else {
            _focusNode.unfocus();
          }
        });
      },
      child: Container(
        width: 48,
        height: 48,
        decoration: BoxDecoration(
          color: const Color(0xFFF1F5F9),
          borderRadius: BorderRadius.circular(24),
        ),
        child: Icon(
          _isTypingMode ? Icons.mic_none_outlined : Icons.keyboard_alt_outlined,
          color: const Color(0xFF64748B),
          size: 24,
        ),
      ),
    );
  }

  Widget _buildTextField() {
    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFFF1F5F9),
        borderRadius: BorderRadius.circular(24),
      ),
      child: TextField(
        controller: _textController,
        focusNode: _focusNode,
        enabled: !isProcessing,
        maxLines: null,
        decoration: const InputDecoration(
          hintText: 'Type a command...',
          hintStyle: TextStyle(color: Color(0xFF94A3B8), fontSize: 16),
          border: InputBorder.none,
          contentPadding: EdgeInsets.symmetric(horizontal: 20, vertical: 12),
        ),
        onChanged: (text) => setState(() {}),
        onSubmitted: (text) {
          if (!isProcessing) _sendCommand(text);
        },
      ),
    );
  }

  Widget _buildMicButton() {
    final theme = Theme.of(context);
    return AnimatedBuilder(
      animation: _pulseAnimation,
      builder: (context, child) {
        return Transform.scale(
          scale: isListening ? _pulseAnimation.value : 1.0,
          child: GestureDetector(
            onTap: _toggleListening,
            child: Container(
              height: 48,
              decoration: BoxDecoration(
                color: isListening ? const Color(0xFFEF4444) : theme.colorScheme.primary,
                borderRadius: BorderRadius.circular(24),
                boxShadow: [
                  BoxShadow(
                    color: (isListening ? const Color(0xFFEF4444) : theme.colorScheme.primary).withOpacity(0.3),
                    blurRadius: 8,
                    offset: const Offset(0, 2),
                  ),
                ],
              ),
              child: Icon(
                isListening ? Icons.stop : Icons.mic,
                color: Colors.white,
                size: 24,
              ),
            ),
          ),
        );
      },
    );
  }

  Widget _buildSendButton() {
    final canSend = _textController.text.trim().isNotEmpty && !isProcessing;
    return GestureDetector(
      onTap: () {
        if (canSend) {
          _sendCommand(_textController.text);
        }
      },
      child: Container(
        width: 48,
        height: 48,
        decoration: BoxDecoration(
          color: canSend ? Theme.of(context).colorScheme.secondary : const Color(0xFFE2E8F0),
          borderRadius: BorderRadius.circular(24),
        ),
        child: Icon(
          Icons.send,
          color: canSend ? Colors.black87 : const Color(0xFF94A3B8),
          size: 24,
        ),
      ),
    );
  }

  Widget _buildListeningIndicator() {
    return Container(
      margin: const EdgeInsets.only(bottom: 12),
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: Theme.of(context).colorScheme.primary.withOpacity(0.1),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 8,
            height: 8,
            decoration: const BoxDecoration(color: Color(0xFFEF4444), shape: BoxShape.circle),
          ),
          const SizedBox(width: 8),
          Text(
            _currentSpeechText.isEmpty ? 'Listening...' : _currentSpeechText,
            style: TextStyle(
              color: Theme.of(context).colorScheme.primary,
              fontSize: 14,
              fontWeight: FontWeight.w500,
            ),
          ),
        ],
      ),
    );
  }

  void _showSettings() {
    showModalBottomSheet(
      context: context,
      backgroundColor: Colors.transparent,
      builder: (context) => _buildSettingsSheet(),
    );
  }

  Widget _buildSettingsSheet() {
    return Container(
      decoration: const BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
      ),
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          Container(
            width: 40,
            height: 4,
            margin: const EdgeInsets.only(top: 12),
            decoration: BoxDecoration(
              color: const Color(0xFFE2E8F0),
              borderRadius: BorderRadius.circular(2),
            ),
          ),
          const Padding(
            padding: EdgeInsets.all(20),
            child: Text(
              'Settings',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600, color: Color(0xFF1E293B)),
            ),
          ),
          _buildSettingItem(
            'Clear Command History',
            Icons.delete_outline,
            () {
              Navigator.pop(context);
              setState(() => _messages.clear());
            },
          ),
          _buildSettingItem(
            'Stop Speaking',
            Icons.volume_off_outlined,
            () {
              Navigator.pop(context);
              _flutterTts.stop();
            },
          ),
          const SizedBox(height: 20),
        ],
      ),
    );
  }

  Widget _buildSettingItem(String title, IconData icon, VoidCallback onTap) {
    return ListTile(
      leading: Icon(icon, color: const Color(0xFF64748B)),
      title: Text(title, style: const TextStyle(color: Color(0xFF1E293B), fontSize: 16)),
      onTap: onTap,
    );
  }

  String _formatTime(DateTime dateTime) {
    final now = DateTime.now();
    final difference = now.difference(dateTime);

    if (difference.inMinutes < 1) return 'Just now';
    if (difference.inHours < 1) return '${difference.inMinutes}m ago';
    if (difference.inDays < 1) return '${difference.inHours}h ago';
    return '${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
  }
}

class ChatMessage {
  final String text;
  final bool isUser;
  final DateTime timestamp;
  final bool isProcessing;

  ChatMessage({
    required this.text,
    required this.isUser,
    required this.timestamp,
    this.isProcessing = false,
  });
}
































// import 'package:flutter/material.dart';
// import 'package:flutter/services.dart';
// import 'package:speech_to_text/speech_to_text.dart' as stt;
// import 'package:flutter_tts/flutter_tts.dart';
// import 'package:google_generative_ai/google_generative_ai.dart';
// import 'package:permission_handler/permission_handler.dart';
// import 'dart:convert';

// // Import the new MQTT service
// import 'mqtt_service.dart';

// // Replace with your actual Gemini API key
// const String GEMINI_API_KEY = 'AIzaSyA-DPDVogtWsF5csfpe9XiUVDJORizwEkY'; // IMPORTANT: Replace with your key

// void main() {
//   runApp(const RoverControlApp());
// }

// class RoverControlApp extends StatelessWidget {
//   const RoverControlApp({super.key});

//   @override
//   Widget build(BuildContext context) {
//     return MaterialApp(
//       title: 'Rover Control',
//       theme: ThemeData(
//         useMaterial3: true,
//         colorScheme: ColorScheme.light(
//           primary: const Color(0xFF0D47A1), // A deeper blue for a more technical feel
//           secondary: const Color(0xFFF9A825), // A gold/yellow for accents
//           surface: Colors.white,
//           background: const Color(0xFFF8FAFC),
//         ),
//         fontFamily: 'SF Pro Display',
//       ),
//       debugShowCheckedModeBanner: false,
//       home: const ChatScreen(),
//     );
//   }
// }

// class ChatScreen extends StatefulWidget {
//   const ChatScreen({super.key});

//   @override
//   State<ChatScreen> createState() => _ChatScreenState();
// }

// class _ChatScreenState extends State<ChatScreen> with TickerProviderStateMixin {
//   final ScrollController _scrollController = ScrollController();
//   final TextEditingController _textController = TextEditingController();
//   final FocusNode _focusNode = FocusNode();

//   bool isListening = false;
//   bool isProcessing = false;
//   bool isSpeaking = false;
//   bool _isTypingMode = false;

//   String _currentSpeechText = '';
//   final List<ChatMessage> _messages = [];

//   late stt.SpeechToText _speech;
//   late FlutterTts _flutterTts;
//   late GenerativeModel _model;
  
//   // --- MQTT Service Integration ---
//   late final MqttService _mqttService;

//   late AnimationController _pulseController;
//   late Animation<double> _pulseAnimation;

//   @override
//   void initState() {
//     super.initState();
//     _initializeServices();
//     _setupAnimations();
//   }

//   void _initializeServices() {
//     _speech = stt.SpeechToText();
//     _initializeTts();
//     _initializeGemini();
//     _requestPermissions();

//     // --- Initialize and connect MQTT Service ---
//     _mqttService = MqttService();
//     _mqttService.connect();
//     // Listen to connection state changes to rebuild the UI
//     _mqttService.connectionState.addListener(_onMqttConnectionStateChanged);
//   }

//   void _onMqttConnectionStateChanged() {
//     // This will trigger a rebuild of the widget to update the connection icon
//     if (mounted) {
//       setState(() {});
//     }
//   }

//   void _setupAnimations() {
//     _pulseController = AnimationController(
//       duration: const Duration(milliseconds: 1000),
//       vsync: this,
//     )..repeat(reverse: true);

//     _pulseAnimation = Tween<double>(
//       begin: 0.8,
//       end: 1.0,
//     ).animate(CurvedAnimation(
//       parent: _pulseController,
//       curve: Curves.easeInOut,
//     ));
//   }

//   void _initializeTts() {
//     _flutterTts = FlutterTts();
//     _flutterTts.setStartHandler(() => setState(() => isSpeaking = true));
//     _flutterTts.setCompletionHandler(() => setState(() => isSpeaking = false));
//     _flutterTts.setErrorHandler((msg) => setState(() => isSpeaking = false));
//     _flutterTts.setSpeechRate(0.5);
//     _flutterTts.setVolume(1.0);
//     _flutterTts.setPitch(1.0);
//   }

//   void _initializeGemini() {
//     _model = GenerativeModel(
//       model: 'gemini-1.5-flash',
//       apiKey: GEMINI_API_KEY,
//     );
//   }

//   Future<void> _requestPermissions() async {
//     await Permission.microphone.request();
//   }

//   @override
//   void dispose() {
//     _pulseController.dispose();
//     _scrollController.dispose();
//     _textController.dispose();
//     _focusNode.dispose();
//     _flutterTts.stop();
//     _mqttService.connectionState.removeListener(_onMqttConnectionStateChanged);
//     _mqttService.disconnect();
//     super.dispose();
//   }

//   Future<void> _startListening() async {
//     if (isProcessing) return;

//     bool available = await _speech.initialize(
//       onStatus: (status) {
//         if (status == 'done' || status == 'notListening') {
//           if (_currentSpeechText.isNotEmpty) {
//             _stopListening();
//           }
//         }
//       },
//       onError: (error) => _stopListening(),
//     );

//     if (available) {
//       setState(() {
//         isListening = true;
//         _currentSpeechText = '';
//       });

//       _speech.listen(
//         onResult: (result) => setState(() => _currentSpeechText = result.recognizedWords),
//         listenFor: const Duration(seconds: 30),
//         pauseFor: const Duration(seconds: 3),
//         partialResults: true,
//       );
//     }
//   }

//   Future<void> _stopListening() async {
//     await _speech.stop();
//     setState(() => isListening = false);

//     if (_currentSpeechText.isNotEmpty) {
//       _sendCommand(_currentSpeechText);
//       _currentSpeechText = '';
//     }
//   }

//   void _toggleListening() {
//     isListening ? _stopListening() : _startListening();
//     HapticFeedback.lightImpact();
//   }

//   Future<void> _sendCommand(String text) async {
//     if (text.trim().isEmpty || isProcessing) return;

//     _textController.clear();
//     final userMessage = ChatMessage(
//       text: text.trim(),
//       isUser: true,
//       timestamp: DateTime.now(),
//     );

//     setState(() {
//       _messages.add(userMessage);
//       isProcessing = true;
//     });
//     _scrollToBottom();

//     final assistantMessage = ChatMessage(
//       text: '',
//       isUser: false,
//       timestamp: DateTime.now(),
//       isProcessing: true,
//     );
//     setState(() => _messages.add(assistantMessage));
//     _scrollToBottom();

//     try {
//       // The prompt now asks for the original command text for clarity
//       final prompt = '''
//         You are the command interpreter for an autonomous rover.
//         Analyze the following user text and extract a command and any relevant parameters.
//         Respond ONLY with a JSON object.
//         The JSON should have "command" (e.g., "move", "turn", "stop", "status", "scan", "goto"), "parameters", and "original_command".
//         For "goto", the parameter should be "destination" (e.g., "point A", "lab entrance").
//         If you cannot determine a valid command, respond with {"command": "unknown", "parameters": {}, "original_command": "$text"}.
//         User text: "$text"
//       ''';

//       final content = [Content.text(prompt)];
//       final response = await _model.generateContent(content);
      
//       String rawResponse = response.text ?? '{"command": "error", "parameters": {}}';
//       rawResponse = rawResponse.replaceAll('```json', '').replaceAll('```', '').trim();

//       final Map<String, dynamic> commandJson = jsonDecode(rawResponse);
//       final String command = commandJson['command'] ?? 'unknown';
//       final String originalCommand = commandJson['original_command'] ?? text;

//       String confirmationText;
//       if (command != 'unknown' && command != 'error') {
//         // --- MQTT PUBLISH ---
//         // Send the original, human-readable command to the rover.
//         // The rover's Jetson Nano will be responsible for parsing this.
//         _mqttService.publish(originalCommand);
//         // --- END MQTT PUBLISH ---

//         confirmationText = "Sending command to rover: \"$originalCommand\"";
//       } else {
//         confirmationText = "Sorry, I didn't understand that command. Please try again.";
//       }

//       setState(() {
//         _messages.last = ChatMessage(
//           text: confirmationText,
//           isUser: false,
//           timestamp: DateTime.now(),
//         );
//         isProcessing = false;
//       });

//       _scrollToBottom();
//       await _flutterTts.speak(confirmationText);

//     } catch (e) {
//       print('Error processing command: $e');
//       final errorText = 'Sorry, an error occurred while interpreting your command.';
//       setState(() {
//         _messages.last = ChatMessage(
//           text: errorText,
//           isUser: false,
//           timestamp: DateTime.now(),
//         );
//         isProcessing = false;
//       });
//       _scrollToBottom();
//       await _flutterTts.speak(errorText);
//     }
//   }

//   void _scrollToBottom() {
//     WidgetsBinding.instance.addPostFrameCallback((_) {
//       if (_scrollController.hasClients) {
//         _scrollController.animateTo(
//           _scrollController.position.maxScrollExtent,
//           duration: const Duration(milliseconds: 300),
//           curve: Curves.easeOut,
//         );
//       }
//     });
//   }

//   @override
//   Widget build(BuildContext context) {
//     return Scaffold(
//       backgroundColor: Theme.of(context).colorScheme.background,
//       appBar: AppBar(
//         backgroundColor: Colors.white,
//         elevation: 1,
//         shadowColor: Colors.black.withOpacity(0.1),
//         title: const Text(
//           'Rover Voice Command',
//           style: TextStyle(
//             color: Color(0xFF1E293B),
//             fontSize: 18,
//             fontWeight: FontWeight.w600,
//           ),
//         ),
//         centerTitle: true,
//         actions: [
//           // --- MQTT Connection Status Indicator ---
//           Padding(
//             padding: const EdgeInsets.only(right: 8.0),
//             child: _buildMqttStatusIcon(),
//           ),
//           IconButton(
//             icon: const Icon(Icons.settings_outlined, color: Color(0xFF64748B)),
//             onPressed: _showSettings,
//           ),
//         ],
//       ),
//       body: Column(
//         children: [
//           Expanded(
//             child: _messages.isEmpty ? _buildEmptyState() : _buildMessageList(),
//           ),
//           _buildInputArea(),
//         ],
//       ),
//     );
//   }

//   // --- New widget for MQTT status icon ---
//   Widget _buildMqttStatusIcon() {
//     IconData icon;
//     Color color;
//     String tooltip;

//     switch (_mqttService.connectionState.value) {
//       case MqttConnectionStatus.connected:
//         icon = Icons.cloud_done_rounded;
//         color = Colors.green;
//         tooltip = 'MQTT Connected';
//         break;
//       case MqttConnectionStatus.connecting:
//         icon = Icons.cloud_upload_rounded;
//         color = Colors.orange;
//         tooltip = 'MQTT Connecting';
//         break;
//       case MqttConnectionStatus.disconnected:
//         icon = Icons.cloud_off_rounded;
//         color = Colors.red;
//         tooltip = 'MQTT Disconnected';
//         break;
//     }
//     return Tooltip(
//       message: tooltip,
//       child: Icon(icon, color: color),
//     );
//   }


//   Widget _buildEmptyState() {
//     return Center(
//       child: Column(
//         mainAxisAlignment: MainAxisAlignment.center,
//         children: [
//           Container(
//             width: 80,
//             height: 80,
//             decoration: BoxDecoration(
//               color: Theme.of(context).colorScheme.primary.withOpacity(0.1),
//               borderRadius: BorderRadius.circular(40),
//             ),
//             child: Icon(
//               Icons.smart_toy_outlined,
//               size: 40,
//               color: Theme.of(context).colorScheme.primary,
//             ),
//           ),
//           const SizedBox(height: 24),
//           const Text(
//             'Rover is Ready',
//             style: TextStyle(
//               fontSize: 22,
//               fontWeight: FontWeight.w600,
//               color: Color(0xFF1E293B),
//             ),
//           ),
//           const SizedBox(height: 8),
//           const Text(
//             'Tap the microphone to issue a command',
//             style: TextStyle(
//               fontSize: 16,
//               color: Color(0xFF64748B),
//             ),
//             textAlign: TextAlign.center,
//           ),
//         ],
//       ),
//     );
//   }

//   Widget _buildMessageList() {
//     return ListView.builder(
//       controller: _scrollController,
//       padding: const EdgeInsets.all(16),
//       itemCount: _messages.length,
//       itemBuilder: (context, index) {
//         final message = _messages[index];
//         return _buildMessageBubble(message);
//       },
//     );
//   }

//   Widget _buildMessageBubble(ChatMessage message) {
//     final isUser = message.isUser;
//     final theme = Theme.of(context);
    
//     return Container(
//       margin: const EdgeInsets.only(bottom: 16),
//       child: Row(
//         crossAxisAlignment: CrossAxisAlignment.start,
//         mainAxisAlignment: isUser ? MainAxisAlignment.end : MainAxisAlignment.start,
//         children: [
//           if (!isUser) ...[
//             Container(
//               width: 32,
//               height: 32,
//               decoration: BoxDecoration(
//                 color: theme.colorScheme.primary,
//                 borderRadius: BorderRadius.circular(16),
//               ),
//               child: const Icon(Icons.smart_toy, color: Colors.white, size: 18),
//             ),
//             const SizedBox(width: 12),
//           ],
//           Flexible(
//             child: Column(
//               crossAxisAlignment: isUser ? CrossAxisAlignment.end : CrossAxisAlignment.start,
//               children: [
//                 Container(
//                   padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
//                   decoration: BoxDecoration(
//                     color: isUser ? theme.colorScheme.primary : Colors.white,
//                     borderRadius: BorderRadius.circular(18).copyWith(
//                       bottomLeft: isUser ? const Radius.circular(18) : const Radius.circular(4),
//                       bottomRight: isUser ? const Radius.circular(4) : const Radius.circular(18),
//                     ),
//                     boxShadow: [
//                       BoxShadow(
//                         color: Colors.black.withOpacity(0.05),
//                         blurRadius: 10,
//                         offset: const Offset(0, 2),
//                       ),
//                     ],
//                   ),
//                   child: message.isProcessing ? _buildProcessingIndicator() : Text(
//                     message.text,
//                     style: TextStyle(
//                       color: isUser ? Colors.white : const Color(0xFF1E293B),
//                       fontSize: 16,
//                       height: 1.4,
//                     ),
//                   ),
//                 ),
//                 const SizedBox(height: 4),
//                 Text(
//                   _formatTime(message.timestamp),
//                   style: const TextStyle(color: Color(0xFF94A3B8), fontSize: 12),
//                 ),
//               ],
//             ),
//           ),
//           if (isUser) ...[
//             const SizedBox(width: 12),
//             Container(
//               width: 32,
//               height: 32,
//               decoration: BoxDecoration(
//                 color: const Color(0xFF64748B),
//                 borderRadius: BorderRadius.circular(16),
//               ),
//               child: const Icon(Icons.person, color: Colors.white, size: 18),
//             ),
//           ],
//         ],
//       ),
//     );
//   }

//   Widget _buildProcessingIndicator() {
//     return Row(
//       mainAxisSize: MainAxisSize.min,
//       children: [
//         SizedBox(
//           width: 16,
//           height: 16,
//           child: CircularProgressIndicator(
//             strokeWidth: 2,
//             valueColor: AlwaysStoppedAnimation<Color>(const Color(0xFF64748B).withOpacity(0.6)),
//           ),
//         ),
//         const SizedBox(width: 8),
//         const Text(
//           'Interpreting command...',
//           style: TextStyle(
//             color: Color(0xFF64748B),
//             fontSize: 16,
//             fontStyle: FontStyle.italic,
//           ),
//         ),
//       ],
//     );
//   }

//   Widget _buildInputArea() {
//     return Container(
//       padding: const EdgeInsets.all(16),
//       decoration: BoxDecoration(
//         color: Colors.white,
//         boxShadow: [
//           BoxShadow(
//             color: Colors.black.withOpacity(0.05),
//             blurRadius: 10,
//             offset: const Offset(0, -2),
//           ),
//         ],
//       ),
//       child: SafeArea(
//         child: Column(
//           crossAxisAlignment: CrossAxisAlignment.start,
//           children: [
//             if (isListening) _buildListeningIndicator(),
//             Row(
//               children: [
//                 _buildModeToggleButton(),
//                 const SizedBox(width: 8),
//                 Expanded(
//                   child: _isTypingMode ? _buildTextField() : _buildMicButton(),
//                 ),
//                 const SizedBox(width: 8),
//                 _isTypingMode ? _buildSendButton() : const SizedBox(width: 48),
//               ],
//             ),
//           ],
//         ),
//       ),
//     );
//   }

//   Widget _buildModeToggleButton() {
//     return GestureDetector(
//       onTap: () {
//         if (isListening) _stopListening();
//         setState(() {
//           _isTypingMode = !_isTypingMode;
//           if (_isTypingMode) {
//             _focusNode.requestFocus();
//           } else {
//             _focusNode.unfocus();
//           }
//         });
//       },
//       child: Container(
//         width: 48,
//         height: 48,
//         decoration: BoxDecoration(
//           color: const Color(0xFFF1F5F9),
//           borderRadius: BorderRadius.circular(24),
//         ),
//         child: Icon(
//           _isTypingMode ? Icons.mic_none_outlined : Icons.keyboard_alt_outlined,
//           color: const Color(0xFF64748B),
//           size: 24,
//         ),
//       ),
//     );
//   }

//   Widget _buildTextField() {
//     return Container(
//       decoration: BoxDecoration(
//         color: const Color(0xFFF1F5F9),
//         borderRadius: BorderRadius.circular(24),
//       ),
//       child: TextField(
//         controller: _textController,
//         focusNode: _focusNode,
//         enabled: !isProcessing,
//         maxLines: null,
//         decoration: const InputDecoration(
//           hintText: 'Type a command...',
//           hintStyle: TextStyle(color: Color(0xFF94A3B8), fontSize: 16),
//           border: InputBorder.none,
//           contentPadding: EdgeInsets.symmetric(horizontal: 20, vertical: 12),
//         ),
//         onChanged: (text) => setState(() {}),
//         onSubmitted: (text) {
//           if (!isProcessing) _sendCommand(text);
//         },
//       ),
//     );
//   }

//   Widget _buildMicButton() {
//     final theme = Theme.of(context);
//     return AnimatedBuilder(
//       animation: _pulseAnimation,
//       builder: (context, child) {
//         return Transform.scale(
//           scale: isListening ? _pulseAnimation.value : 1.0,
//           child: GestureDetector(
//             onTap: _toggleListening,
//             child: Container(
//               height: 48,
//               decoration: BoxDecoration(
//                 color: isListening ? const Color(0xFFEF4444) : theme.colorScheme.primary,
//                 borderRadius: BorderRadius.circular(24),
//                 boxShadow: [
//                   BoxShadow(
//                     color: (isListening ? const Color(0xFFEF4444) : theme.colorScheme.primary).withOpacity(0.3),
//                     blurRadius: 8,
//                     offset: const Offset(0, 2),
//                   ),
//                 ],
//               ),
//               child: Icon(
//                 isListening ? Icons.stop : Icons.mic,
//                 color: Colors.white,
//                 size: 24,
//               ),
//             ),
//           ),
//         );
//       },
//     );
//   }

//   Widget _buildSendButton() {
//     final canSend = _textController.text.trim().isNotEmpty && !isProcessing;
//     return GestureDetector(
//       onTap: () {
//         if (canSend) {
//           _sendCommand(_textController.text);
//         }
//       },
//       child: Container(
//         width: 48,
//         height: 48,
//         decoration: BoxDecoration(
//           color: canSend ? Theme.of(context).colorScheme.secondary : const Color(0xFFE2E8F0),
//           borderRadius: BorderRadius.circular(24),
//         ),
//         child: Icon(
//           Icons.send,
//           color: canSend ? Colors.black87 : const Color(0xFF94A3B8),
//           size: 24,
//         ),
//       ),
//     );
//   }

//   Widget _buildListeningIndicator() {
//     return Container(
//       margin: const EdgeInsets.only(bottom: 12),
//       padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
//       decoration: BoxDecoration(
//         color: Theme.of(context).colorScheme.primary.withOpacity(0.1),
//         borderRadius: BorderRadius.circular(20),
//       ),
//       child: Row(
//         mainAxisSize: MainAxisSize.min,
//         children: [
//           Container(
//             width: 8,
//             height: 8,
//             decoration: const BoxDecoration(color: Color(0xFFEF4444), shape: BoxShape.circle),
//           ),
//           const SizedBox(width: 8),
//           Text(
//             _currentSpeechText.isEmpty ? 'Listening...' : _currentSpeechText,
//             style: TextStyle(
//               color: Theme.of(context).colorScheme.primary,
//               fontSize: 14,
//               fontWeight: FontWeight.w500,
//             ),
//           ),
//         ],
//       ),
//     );
//   }

//   void _showSettings() {
//     showModalBottomSheet(
//       context: context,
//       backgroundColor: Colors.transparent,
//       builder: (context) => _buildSettingsSheet(),
//     );
//   }

//   Widget _buildSettingsSheet() {
//     return Container(
//       decoration: const BoxDecoration(
//         color: Colors.white,
//         borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
//       ),
//       child: Column(
//         mainAxisSize: MainAxisSize.min,
//         children: [
//           Container(
//             width: 40,
//             height: 4,
//             margin: const EdgeInsets.only(top: 12),
//             decoration: BoxDecoration(
//               color: const Color(0xFFE2E8F0),
//               borderRadius: BorderRadius.circular(2),
//             ),
//           ),
//           const Padding(
//             padding: EdgeInsets.all(20),
//             child: Text(
//               'Settings',
//               style: TextStyle(fontSize: 18, fontWeight: FontWeight.w600, color: Color(0xFF1E293B)),
//             ),
//           ),
//           _buildSettingItem(
//             'Clear Command History',
//             Icons.delete_outline,
//             () {
//               Navigator.pop(context);
//               setState(() => _messages.clear());
//             },
//           ),
//           _buildSettingItem(
//             'Stop Speaking',
//             Icons.volume_off_outlined,
//             () {
//               Navigator.pop(context);
//               _flutterTts.stop();
//             },
//           ),
//           const SizedBox(height: 20),
//         ],
//       ),
//     );
//   }

//   Widget _buildSettingItem(String title, IconData icon, VoidCallback onTap) {
//     return ListTile(
//       leading: Icon(icon, color: const Color(0xFF64748B)),
//       title: Text(title, style: const TextStyle(color: Color(0xFF1E293B), fontSize: 16)),
//       onTap: onTap,
//     );
//   }

//   String _formatTime(DateTime dateTime) {
//     final now = DateTime.now();
//     final difference = now.difference(dateTime);

//     if (difference.inMinutes < 1) return 'Just now';
//     if (difference.inHours < 1) return '${difference.inMinutes}m ago';
//     if (difference.inDays < 1) return '${difference.inHours}h ago';
//     return '${dateTime.hour.toString().padLeft(2, '0')}:${dateTime.minute.toString().padLeft(2, '0')}';
//   }
// }

// class ChatMessage {
//   final String text;
//   final bool isUser;
//   final DateTime timestamp;
//   final bool isProcessing;

//   ChatMessage({
//     required this.text,
//     required this.isUser,
//     required this.timestamp,
//     this.isProcessing = false,
//   });
// }
