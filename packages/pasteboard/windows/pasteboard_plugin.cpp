#include "include/pasteboard/pasteboard_plugin.h"

#include <flutter/method_channel.h>
#include <flutter/plugin_registrar_windows.h>
#include <flutter/standard_method_codec.h>

#include <Windows.h>
#include <ShlObj.h>

#include <map>
#include <memory>

#include "strconv.h"

#include <algorithm>
#include <cstring>
#include <iterator>
// #include <GdiPlus.h>
#include "include/FreeImage.h"
// #pragma comment(lib, "gdiplus.lib")

// using namespace Gdiplus;
using namespace std;

namespace
{

	// 	GdiplusStartupInput m_Gdistart;
	// 	ULONG_PTR m_GdiplusToken;
	using flutter::EncodableList;
	using flutter::EncodableMap;
	using flutter::EncodableValue;

	constexpr STGMEDIUM kNullStorageMedium = {TYMED_NULL, nullptr, nullptr};

	void printVector(const vector<uint8_t> &v)
	{
		cout << "vector content         : ";
		for_each(v.cbegin(), v.cend(), [](const uint8_t &val) -> void
				 { cout << val; });
		cout << endl;
	}
	std::string getArgumentString(const EncodableMap *arguments, string key)
	{
		std::string value;
		auto value_auto = arguments->find(EncodableValue(key));
		if (value_auto != arguments->end())
		{
			if (std::holds_alternative<std::string>(value_auto->second))
			{
				value = std::get<std::string>(value_auto->second);
			}
			else
			{
			}
		}
		return value;
	}

	int copyBitmapToClipboard(std::vector<uint8_t> bitmapBuffer, size_t buflen)
	{
		HGLOBAL hResult;
		if (!OpenClipboard(NULL))
			return 1;
		if (!EmptyClipboard())
			return 1;
		// cout << buflen << endl;
		buflen -= sizeof(BITMAPFILEHEADER);
		// cout << buflen << endl;
		hResult = GlobalAlloc(GMEM_MOVEABLE, buflen);
		//  if (hResult == NULL) return PASTE_DATA_ERROR;
		// cout << "bitmapheader size  " << sizeof(BITMAPFILEHEADER) << endl;
		bitmapBuffer.erase(bitmapBuffer.begin(), bitmapBuffer.begin() + sizeof(BITMAPFILEHEADER));
		memcpy(GlobalLock(hResult), static_cast<void *>(bitmapBuffer.data()), buflen);
		GlobalUnlock(hResult);

		HANDLE hh = SetClipboardData(CF_DIB, hResult);
		if (hh == NULL)
		{
			CloseClipboard();
			DWORD erroe = GetLastError();
			cout << erroe << endl;
			// return PASTE_PASTE_ERROR;
			return 1;
		}
		// std::string icon = "aaa.bmp";
		// wchar_t *wc = new wchar_t[icon.size()];
		// swprintf(wc, 100, L"%S", icon.c_str());
		// HBITMAP pDstImg = static_cast<HBITMAP>(LoadImage(GetModuleHandle(nullptr), wc, IMAGE_BITMAP, 0, 0, LR_LOADFROMFILE));
		// SetClipboardData(CF_BITMAP, pDstImg);
		GetLastError();
		// SetClipboardData(CF_TEXT, "swx111");
		// BitmapToDIB();
		// SetClipboardData(CF_DIB, hResult);

		CloseClipboard();
		GlobalFree(hResult);
		//  return PASTE_WE_DID_IT_YAY;
		return 0;
	}

	STGMEDIUM CreateStorageForFileNames(const std::vector<std::string> &filenames)
	{
		// CF_HDROP clipboard format consists of DROPFILES structure, a series of file
		// names including the terminating null character and the additional null
		// character at the tail to terminate the array.
		// For example,
		//| DROPFILES | FILENAME 1 | NULL | ... | FILENAME n | NULL | NULL |
		// For more details, please refer to
		// https://docs.microsoft.com/en-us/windows/desktop/shell/clipboard#cf_hdrop

		if (filenames.empty())
			return kNullStorageMedium;

		const size_t kDropFilesHeaderSizeInBytes = sizeof(DROPFILES);
		size_t total_bytes = kDropFilesHeaderSizeInBytes;
		for (const auto &filename : filenames)
		{
			// Allocate memory of the filename's length including the null
			// character.
			total_bytes += (filename.length() + 1) * sizeof(wchar_t);
		}
		// |data| needs to be terminated by an additional null character.
		total_bytes += sizeof(wchar_t);

		// GHND combines GMEM_MOVEABLE and GMEM_ZEROINIT, and GMEM_ZEROINIT
		// initializes memory contents to zero.
		HANDLE hdata = GlobalAlloc(GHND, total_bytes);

		auto *drop_files = (DROPFILES *)GlobalLock(hdata);
		drop_files->pFiles = sizeof(DROPFILES);
		drop_files->fWide = TRUE;

		auto *data = reinterpret_cast<wchar_t *>(
			reinterpret_cast<BYTE *>(drop_files) + kDropFilesHeaderSizeInBytes);

		size_t next_filename_offset = 0;
		for (const auto &filename : filenames)
		{
			auto wide_filename = utf8_to_wide(filename);
			wcsncpy_s(data + next_filename_offset,
					  wide_filename.length() + 1,
					  wide_filename.c_str(),
					  wide_filename.length() + 1);
			// Skip the terminating null character of the filename.
			next_filename_offset += wide_filename.length() + 1;
		}

		STGMEDIUM storage;
		storage.tymed = TYMED_HGLOBAL;
		storage.hGlobal = hdata;
		storage.pUnkForRelease = nullptr;

		GlobalUnlock(hdata);
		return storage;
	}

	PBITMAPINFO CreateBitmapInfoStruct(HBITMAP hBmp)
	{
		BITMAP bmp;
		PBITMAPINFO pbmi;
		WORD cClrBits;

		// Retrieve the bitmap color format, width, and height.
		GetObject(hBmp, sizeof(BITMAP), (LPSTR)&bmp);

		// Convert the color format to a count of bits.
		cClrBits = (WORD)(bmp.bmPlanes * bmp.bmBitsPixel);
		if (cClrBits == 1)
			cClrBits = 1;
		else if (cClrBits <= 4)
			cClrBits = 4;
		else if (cClrBits <= 8)
			cClrBits = 8;
		else if (cClrBits <= 16)
			cClrBits = 16;
		else if (cClrBits <= 24)
			cClrBits = 24;
		else
			cClrBits = 32;

		// Allocate memory for the BITMAPINFO structure. (This structure
		// contains a BITMAPINFOHEADER structure and an array of RGBQUAD
		// data structures.)

		if (cClrBits < 24)
			pbmi = (PBITMAPINFO)LocalAlloc(
				LPTR, sizeof(BITMAPINFOHEADER) + sizeof(RGBQUAD) * int(1 << cClrBits));

		// There is no RGBQUAD array for these formats: 24-bit-per-pixel or
		// 32-bit-per-pixel

		else
			pbmi = (PBITMAPINFO)LocalAlloc(LPTR, sizeof(BITMAPINFOHEADER));

		// Initialize the fields in the BITMAPINFO structure.

		pbmi->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
		pbmi->bmiHeader.biWidth = bmp.bmWidth;
		pbmi->bmiHeader.biHeight = bmp.bmHeight;
		pbmi->bmiHeader.biPlanes = bmp.bmPlanes;
		pbmi->bmiHeader.biBitCount = bmp.bmBitsPixel;
		if (cClrBits < 24)
			pbmi->bmiHeader.biClrUsed = (1 << cClrBits);

		// If the bitmap is not compressed, set the BI_RGB flag.
		pbmi->bmiHeader.biCompression = BI_RGB;

		// Compute the number of bytes in the array of color
		// indices and store the result in biSizeImage.
		// The width must be DWORD aligned unless the bitmap is RLE
		// compressed.
		pbmi->bmiHeader.biSizeImage =
			((pbmi->bmiHeader.biWidth * cClrBits + 31) & ~31) / 8 *
			pbmi->bmiHeader.biHeight;
		// Set biClrImportant to 0, indicating that all of the
		// device colors are important.
		pbmi->bmiHeader.biClrImportant = 0;
		return pbmi;
	}

	void CreateBMPFile(LPCTSTR pszFile, HBITMAP hBMP)
	{
		HANDLE hf;				// file handle
		BITMAPFILEHEADER hdr;	// bitmap file-header
		PBITMAPINFOHEADER pbih; // bitmap info-header
		LPBYTE lpBits;			// memory pointer
		DWORD cb;				// incremental count of bytes
		BYTE *hp;				// byte pointer
		DWORD dwTmp;
		PBITMAPINFO pbi;
		HDC hDC;

		hDC = CreateCompatibleDC(GetWindowDC(GetDesktopWindow()));
		SelectObject(hDC, hBMP);

		pbi = CreateBitmapInfoStruct(hBMP);

		pbih = (PBITMAPINFOHEADER)pbi;
		lpBits = (LPBYTE)GlobalAlloc(GMEM_FIXED, pbih->biSizeImage);

		assert(lpBits);

		// Retrieve the color table (RGBQUAD array) and the bits
		// (array of palette indices) from the DIB.
		GetDIBits(hDC, hBMP, 0, (WORD)pbih->biHeight, lpBits, pbi,
				  DIB_RGB_COLORS);

		// Create the .BMP file.
		hf = CreateFile(pszFile, GENERIC_READ | GENERIC_WRITE, (DWORD)0, nullptr,
						CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		assert(hf != INVALID_HANDLE_VALUE);

		hdr.bfType = 0x4d42; // 0x42 = "B" 0x4d = "M"
		// Compute the size of the entire file.
		hdr.bfSize = (DWORD)(sizeof(BITMAPFILEHEADER) + pbih->biSize +
							 pbih->biClrUsed * sizeof(RGBQUAD) + pbih->biSizeImage);
		hdr.bfReserved1 = 0;
		hdr.bfReserved2 = 0;

		// Compute the offset to the array of color indices.
		hdr.bfOffBits = (DWORD)sizeof(BITMAPFILEHEADER) + pbih->biSize +
						pbih->biClrUsed * sizeof(RGBQUAD);

		// Copy the BITMAPFILEHEADER into the .BMP file.
		WriteFile(hf, (LPVOID)&hdr, sizeof(BITMAPFILEHEADER), (LPDWORD)&dwTmp,
				  nullptr);

		// Copy the BITMAPINFOHEADER and RGBQUAD array into the file.
		WriteFile(hf, (LPVOID)pbih,
				  sizeof(BITMAPINFOHEADER) + pbih->biClrUsed * sizeof(RGBQUAD),
				  (LPDWORD)&dwTmp, (nullptr));

		// Copy the array of color indices into the .BMP file.
		cb = pbih->biSizeImage;
		hp = lpBits;
		WriteFile(hf, (LPSTR)hp, (int)cb, (LPDWORD)&dwTmp, nullptr);

		// Close the .BMP file.
		CloseHandle(hf);

		// Free memory.
		GlobalFree((HGLOBAL)lpBits);
	}

	void CreateBitmapHeaderWithColorDepth(LONG width, LONG height, WORD color_depth,
										  BITMAPINFOHEADER *hdr)
	{
		// These values are shared with gfx::PlatformDevice.
		hdr->biSize = sizeof(BITMAPINFOHEADER);
		hdr->biWidth = width;
		hdr->biHeight = -height; // Minus means top-down bitmap.
		hdr->biPlanes = 1;
		hdr->biBitCount = color_depth;
		hdr->biCompression = BI_RGB; // No compression.
		hdr->biSizeImage = 0;
		hdr->biXPelsPerMeter = 1;
		hdr->biYPelsPerMeter = 1;
		hdr->biClrUsed = 0;
		hdr->biClrImportant = 0;
	}

	string Utf8ToGbk(const char *src_str)
	{
		int len = MultiByteToWideChar(CP_UTF8, 0, src_str, -1, NULL, 0);
		wchar_t *wszGBK = new wchar_t[len + 1];
		memset(wszGBK, 0, len * 2 + 2);
		MultiByteToWideChar(CP_UTF8, 0, src_str, -1, wszGBK, len);
		len = WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, NULL, 0, NULL, NULL);
		char *szGBK = new char[len + 1];
		memset(szGBK, 0, len + 1);
		WideCharToMultiByte(CP_ACP, 0, wszGBK, -1, szGBK, len, NULL, NULL);
		string strTemp(szGBK);
		if (wszGBK)
			delete[] wszGBK;
		if (szGBK)
			delete[] szGBK;
		return strTemp;
	}

	std::wstring stringToWstring(const std::string &str)
	{
		LPCSTR pszSrc = str.c_str();
		int nLen = MultiByteToWideChar(CP_ACP, 0, pszSrc, -1, NULL, 0);
		if (nLen == 0)
			return std::wstring(L"");

		wchar_t *pwszDst = new wchar_t[nLen];
		if (!pwszDst)
			return std::wstring(L"");

		MultiByteToWideChar(CP_ACP, 0, pszSrc, -1, pwszDst, nLen);
		std::wstring wstr(pwszDst);
		delete[] pwszDst;
		pwszDst = NULL;

		return wstr;
	}

	HBITMAP CreateHBitmapXRGB8888(int width, int height, HANDLE shared_section,
								  void **data)
	{
		if (width == 0 || height == 0)
		{
			width = 1;
			height = 1;
		}
		BITMAPINFOHEADER hdr = {0};
		CreateBitmapHeaderWithColorDepth(width, height, 32, &hdr);
		HBITMAP hbitmap =
			CreateDIBSection(nullptr, reinterpret_cast<const BITMAPINFO *>(&hdr), 0,
							 data, shared_section, 0);
		return hbitmap;
	}

	int GetDIBBitDepth(HANDLE hData)
	{
		BITMAPINFO *bmi = (BITMAPINFO *)GlobalLock(hData);
		if (bmi != NULL)
		{
			int bitDepth = bmi->bmiHeader.biBitCount;
			GlobalUnlock(hData);
			return bitDepth;
		}
		return 0; // Failed to get bit depth
	}

	int GetDIBWidth(HANDLE hData)
	{
		BITMAPINFO *bmi = (BITMAPINFO *)GlobalLock(hData);
		if (bmi != NULL)
		{
			int width = bmi->bmiHeader.biWidth;
			GlobalUnlock(hData);
			return width;
		}
		return 0; // Failed to get width
	}

	int GetDIBHeight(HANDLE hData)
	{
		BITMAPINFO *bmi = (BITMAPINFO *)GlobalLock(hData);
		if (bmi != NULL)
		{
			int height = abs(bmi->bmiHeader.biHeight); // Make sure it's positive (DIB is upside-down)
			GlobalUnlock(hData);
			return height;
		}
		return 0; // Failed to get height
	}

	bool save2png(std::string filePath)
	{
		HDC hdc = CreateDCA("DISPLAY", NULL, NULL, NULL);
		if (!OpenClipboard(nullptr))
		{
			std::cout << "cannot open clipboard" << std::endl;
			return false;
		}
		int bitDepth = 24;
		int bitWidth = 0;
		int bitHeight = 0;
		if (IsClipboardFormatAvailable(CF_DIB))
		{

			HANDLE hData = GetClipboardData(CF_DIB);
			if (hData != NULL)
			{
				bitDepth = GetDIBBitDepth(hData);
				bitWidth = GetDIBWidth(hData);
				bitHeight = GetDIBHeight(hData);
			}
		}
		HBITMAP hBitmap = nullptr;
		if (IsClipboardFormatAvailable(CF_BITMAP))
		{
			hBitmap = static_cast<HBITMAP>(GetClipboardData(CF_BITMAP));
		}
		if (hBitmap != nullptr)
		{
			FIBITMAP *dib_new = FreeImage_Allocate(bitWidth, bitHeight, bitDepth);
			GetDIBits(hdc, hBitmap, 0, FreeImage_GetHeight(dib_new),
					  FreeImage_GetBits(dib_new), FreeImage_GetInfo(dib_new), DIB_RGB_COLORS);
			int nColors = FreeImage_GetColorsUsed(dib_new);
			FreeImage_GetInfoHeader(dib_new)->biClrUsed = nColors;
			FreeImage_GetInfoHeader(dib_new)->biClrImportant = nColors;
			FreeImage_Save(FIF_PNG, dib_new, filePath.c_str());
			FreeImage_Unload(dib_new);
			DeleteObject(hBitmap);
		}
		CloseClipboard();
		return true;
	}

	// A scoper to manage acquiring and automatically releasing the clipboard.
	class ScopedClipboard
	{
	public:
		ScopedClipboard() : opened_(false) {}

		~ScopedClipboard()
		{
			if (opened_)
				Release();
		}

		bool Acquire(HWND owner)
		{
			const int kMaxAttemptsToOpenClipboard = 5;

			if (opened_)
			{
				return false;
			}

			for (int attempts = 0; attempts < kMaxAttemptsToOpenClipboard; ++attempts)
			{
				if (::OpenClipboard(owner))
				{
					opened_ = true;
					return true;
				}

				// If we didn't manage to open the clipboard, sleep a bit and be hopeful.
				::Sleep(5);
			}

			// We failed to acquire the clipboard.
			return false;
		}

		void Release()
		{
			if (opened_)
			{
				::CloseClipboard();
				opened_ = false;
			}
		}

	private:
		bool opened_;
	};

	class PasteboardPlugin : public flutter::Plugin
	{
	public:
		static void RegisterWithRegistrar(flutter::PluginRegistrarWindows *registrar);

		PasteboardPlugin();

		virtual ~PasteboardPlugin();

	private:
		// Called when a method is called on this plugin's channel from Dart.
		void HandleMethodCall(
			const flutter::MethodCall<flutter::EncodableValue> &method_call,
			std::unique_ptr<flutter::MethodResult<flutter::EncodableValue>> result);
	};

	// static
	void PasteboardPlugin::RegisterWithRegistrar(
		flutter::PluginRegistrarWindows *registrar)
	{
		auto channel =
			std::make_unique<flutter::MethodChannel<flutter::EncodableValue>>(
				registrar->messenger(), "pasteboard",
				&flutter::StandardMethodCodec::GetInstance());

		auto plugin = std::make_unique<PasteboardPlugin>();

		channel->SetMethodCallHandler(
			[plugin_pointer = plugin.get()](const auto &call, auto result)
			{
				plugin_pointer->HandleMethodCall(call, std::move(result));
			});

		registrar->AddPlugin(std::move(plugin));
	}

	PasteboardPlugin::PasteboardPlugin() {}

	PasteboardPlugin::~PasteboardPlugin() {}

	void PasteboardPlugin::HandleMethodCall(
		const flutter::MethodCall<flutter::EncodableValue> &method_call,
		std::unique_ptr<flutter::MethodResult<flutter::EncodableValue>> result)
	{
		if (method_call.method_name() == "image")
		{
			if (!IsClipboardFormatAvailable(CF_DIB))
			{
				result->Success();
				return;
			}
			ScopedClipboard clipboard;

			if (!clipboard.Acquire(nullptr))
			{
				result->Error("0", "open clipboard failed");
				return;
			}
			// We use a DIB rather than a DDB here since ::GetObject() with the
			// HBITMAP returned from ::GetClipboardData(CF_BITMAP) always reports a
			// color depth of 32bpp.
			auto *bitmap = static_cast<BITMAPINFO *>(::GetClipboardData(CF_DIB));
			if (!bitmap)
			{
				result->Success();
				return;
			}

			int color_table_length = 0;

			// For more information on BITMAPINFOHEADER and biBitCount definition,
			// see https://docs.microsoft.com/en-us/windows/win32/wmdm/-bitmapinfoheader
			switch (bitmap->bmiHeader.biBitCount)
			{
			case 1:
			case 4:
			case 8:
				color_table_length = bitmap->bmiHeader.biClrUsed
										 ? int(bitmap->bmiHeader.biClrUsed)
										 : int(1 << bitmap->bmiHeader.biBitCount);
				break;
			case 16:
			case 32:
				if (bitmap->bmiHeader.biCompression == BI_BITFIELDS)
					color_table_length = 3;
				break;
			case 24:
				break;
			default:
				result->Success();
				return;
			}

			const void *bitmap_bits = reinterpret_cast<const char *>(bitmap) +
									  bitmap->bmiHeader.biSize +
									  color_table_length * sizeof(RGBQUAD);

			void *dst_bits;
			auto dst_hbitmap =
				CreateHBitmapXRGB8888(bitmap->bmiHeader.biWidth,
									  bitmap->bmiHeader.biHeight, nullptr, &dst_bits);

			auto hdc = CreateCompatibleDC(nullptr);
			auto old_hbitmap = static_cast<HBITMAP>(SelectObject(hdc, dst_hbitmap));
			::SetDIBitsToDevice(
				hdc, 0, 0, bitmap->bmiHeader.biWidth, bitmap->bmiHeader.biHeight, 0, 0,
				0, bitmap->bmiHeader.biHeight, bitmap_bits, bitmap, DIB_RGB_COLORS);
			SelectObject(hdc, old_hbitmap);
			DeleteDC(hdc);

			TCHAR path[MAX_PATH];
			GetTempPath(MAX_PATH, path);
			TCHAR name[MAX_PATH];
			GetTempFileName(path, L"pasteboard", false, name);
			CreateBMPFile(name, dst_hbitmap);

			DeleteObject(dst_hbitmap);

			result->Success(flutter::EncodableValue(wide_to_utf8(name)));
		}
		else if (method_call.method_name() == "files")
		{
			if (!IsClipboardFormatAvailable(CF_HDROP))
			{
				result->Success();
				return;
			}
			if (!OpenClipboard(nullptr))
			{
				result->Error("0", "open clipboard failed");
				return;
			}
			auto handle = GetClipboardData(CF_HDROP);
			flutter::EncodableList file_list;
			if (handle)
			{
				auto data = reinterpret_cast<HDROP>(GlobalLock(handle));
				if (data)
				{
					auto files = DragQueryFile(data, 0xFFFFFFFF, nullptr, 0);
					for (unsigned int i = 0; i < files; ++i)
					{
						TCHAR filename[MAX_PATH];
						DragQueryFile(data, i, filename, sizeof(TCHAR) * MAX_PATH);
						std::wstring wide_filename(filename);
						file_list.emplace_back(wide_to_utf8(wide_filename));
					}
				}
			}
			CloseClipboard();
			result->Success(flutter::EncodableValue(file_list));
		}
		else if (method_call.method_name() == "writeFiles")
		{
			auto *arguments = method_call.arguments();
			auto files = std::get_if<std::vector<flutter::EncodableValue>>(arguments);
			if (!files)
			{
				result->Error("0", "files is empty");
				return;
			}
			std::vector<std::string> paths;
			for (const auto &item : *files)
			{
				if (std::holds_alternative<std::string>(item))
				{
					paths.push_back(std::get<std::string>(item));
				}
			}

			if (paths.empty())
			{
				result->Error("0", "files is empty");
				return;
			}

			ScopedClipboard clipboard;
			if (!clipboard.Acquire(nullptr))
			{
				result->Error("0", "failed to open clipboard");
				return;
			}
			EmptyClipboard();
			auto storage = CreateStorageForFileNames(paths);
			if (storage.tymed == TYMED_NULL)
			{
				result->Error("0", "create storage failed");
				return;
			}
			SetClipboardData(CF_HDROP, storage.hGlobal);
			result->Success();
		}
		else if (method_call.method_name() == "html")
		{

			UINT CF_HTML = RegisterClipboardFormatA("HTML Format");
			bool isHTMLFormatAvailable = false;

			if (IsClipboardFormatAvailable(CF_HTML))
			{
				isHTMLFormatAvailable = true;
			}

			if (!isHTMLFormatAvailable)
			{
				result->Success();
				return;
			}

			if (!OpenClipboard(nullptr))
			{
				result->Error("0", "open clipboard failed");
				return;
			}
			HANDLE hClipboardData = GetClipboardData(CF_HTML);
			if (hClipboardData == NULL)
			{
				result->Success();
				CloseClipboard();
				return;
			}
			else
			{
				char *p = (char *)GlobalLock(hClipboardData);
				SIZE_T size = GlobalSize(hClipboardData);
				std::string str;
				str.assign(p, size);
				result->Success(flutter::EncodableValue(str));
				GlobalUnlock(hClipboardData);
			}
			CloseClipboard();
		}
		else if (method_call.method_name() == "writeImage")
		{
			//		auto arguments = method_call.arguments();
			//		//  getArgumentString(arguments,"");
			//
			//		std::cout << arguments << std::endl;
			//		string data;
			//		std::vector<uint8_t> data1;
			//		if (std::holds_alternative<std::string>(*arguments))
			//		{
			//			data = std::get<std::string>(*arguments);
			//		}
			//		// std::vector<uint8_t>
			//
			//		if (std::holds_alternative<std::vector<uint8_t>>(*arguments))
			//		{
			//			data1 = std::get<std::vector<uint8_t>>(*arguments);
			//		}
			//
			//		string str = std::string(data1.data(), data1.data() + data1.size());
			//		str = Utf8ToGbk(str.c_str());
			//		std::cout << str << std::endl;
			//		wstring ss = stringToWstring(str);
			//		Gdiplus::Bitmap *image1 = Gdiplus::Bitmap::FromFile(ss.c_str());
			//		Gdiplus::Bitmap &image = *image1;
			//		HBITMAP hbitmap;
			//		if (image1 == NULL)
			//		{
			//			std::cout << "NULL" << std::endl;
			//			result->Success();
			//			return;
			//		}
			//		image.GetHBITMAP(0, &hbitmap);
			//		if (OpenClipboard(NULL))
			//		{
			//			EmptyClipboard();
			//
			//			DIBSECTION ds;
			//			GetObject(hbitmap, sizeof(DIBSECTION), &ds);
			//
			//			// make sure compression is BI_RGB
			//			ds.dsBmih.biCompression = BI_RGB;
			//
			//			// Convert DIB to DDB
			//			HDC hdc = GetDC(NULL);
			//			HBITMAP hbitmap_ddb = CreateDIBitmap(hdc, &ds.dsBmih, CBM_INIT,
			//												 ds.dsBm.bmBits, (BITMAPINFO *)&ds.dsBmih, DIB_RGB_COLORS);
			//			ReleaseDC(NULL, hdc);
			//
			//			SetClipboardData(CF_BITMAP, hbitmap_ddb);
			//			CloseClipboard();
			//			DeleteObject(hbitmap_ddb);
			//		}
			//
			//		// copyBitmapToClipboard(data1, data1.size());
			result->Success();
		}
		else if (method_call.method_name() == "save2png")
		{
			auto arguments = method_call.arguments();
			std::string filePath;
			if (std::holds_alternative<std::string>(*arguments))
			{
				filePath = std::get<std::string>(*arguments);
				filePath = Utf8ToGbk(filePath.c_str());
			}
			bool res = false;
			try
			{
				res = save2png(filePath);
			}
			catch (exception e)
			{
				res = false;
			}
			result->Success(flutter::EncodableValue(res));
		}
		else
		{
			result->NotImplemented();
		}
	}

} // namespace

void PasteboardPluginRegisterWithRegistrar(
	FlutterDesktopPluginRegistrarRef registrar)
{
	PasteboardPlugin::RegisterWithRegistrar(
		flutter::PluginRegistrarManager::GetInstance()
			->GetRegistrar<flutter::PluginRegistrarWindows>(registrar));
	// GdiplusStartup(&m_GdiplusToken, &m_Gdistart, NULL);
}
