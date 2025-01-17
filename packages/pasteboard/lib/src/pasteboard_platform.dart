import 'dart:typed_data';

abstract class PasteboardPlatform {
  Future<Uint8List?> get image;

  Future<String?> get html;

  Future<void> writeImage(Uint8List? image);

  Future<List<String>> files();

  Future<bool> writeFiles(List<String> files);

  Future<void> writeHtml(String html);

  Future<String?> get text;

  void writeText(String value);

  Future<bool> save2png(String path);
}
