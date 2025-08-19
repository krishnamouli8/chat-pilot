import 'package:flutter/foundation.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'dart:io';

// Enum to represent the connection state for the UI
enum MqttConnectionStatus { connected, disconnected, connecting }

class MqttService {
  // Use a ValueNotifier to allow the UI to listen for connection status changes
  final ValueNotifier<MqttConnectionStatus> connectionState =
      ValueNotifier(MqttConnectionStatus.disconnected);

  late MqttServerClient _client;

  // --- Configuration ---
  final String _server = 'broker.hivemq.com'; // Public broker for testing
  final int _port = 1883;
  final String _topic = 'rover/command'; // Topic to publish commands to
  
  // A unique client ID for the connection
  final String _clientIdentifier = 'flutter_rover_app_${DateTime.now().millisecondsSinceEpoch}';

  // --- Public method to initiate connection ---
  Future<void> connect() async {
    _client = MqttServerClient.withPort(_server, _clientIdentifier, _port);
    _client.logging(on: false); // Disable logging for production
    _client.keepAlivePeriod = 60;
    _client.onConnected = _onConnected;
    _client.onDisconnected = _onDisconnected;
    _client.onUnsubscribed = _onUnsubscribed;
    _client.port = _port;
    _client.secure = false;

    final connMessage = MqttConnectMessage()
        .withClientIdentifier(_clientIdentifier)
        .startClean() // Non-persistent session
        .withWillQos(MqttQos.atLeastOnce);
    
    _client.connectionMessage = connMessage;

    try {
      print('MQTT_CLIENT::Connecting to broker...');
      connectionState.value = MqttConnectionStatus.connecting;
      await _client.connect();
    } on NoConnectionException catch (e) {
      print('MQTT_CLIENT::Client exception - $e');
      _client.disconnect();
      connectionState.value = MqttConnectionStatus.disconnected;
    } on SocketException catch (e) {
      print('MQTT_CLIENT::Socket exception - $e');
      _client.disconnect();
      connectionState.value = MqttConnectionStatus.disconnected;
    }
  }

  // --- Public method to publish a message ---
  void publish(String message) {
    if (_client.connectionStatus!.state == MqttConnectionState.connected) {
      final builder = MqttClientPayloadBuilder();
      builder.addString(message);
      _client.publishMessage(_topic, MqttQos.atLeastOnce, builder.payload!);
      print('MQTT_CLIENT::Published message: $message');
    } else {
      print('MQTT_CLIENT::Cannot publish, client is not connected.');
    }
  }

  // --- Private connection callbacks ---
  void _onConnected() {
    connectionState.value = MqttConnectionStatus.connected;
    print('MQTT_CLIENT::Connected to broker.');
  }

  void _onDisconnected() {
    connectionState.value = MqttConnectionStatus.disconnected;
    print('MQTT_CLIENT::Disconnected from broker.');
  }

  void _onUnsubscribed(String? topic) {
    print('MQTT_CLIENT::Unsubscribed from topic: $topic');
  }

  // --- Public method to disconnect ---
  void disconnect() {
    _client.disconnect();
  }
}
