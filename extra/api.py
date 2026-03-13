#!/usr/bin/env python3

# Copyright (c) 2025-2026 Mark R V Murray.
#
# SPDX-License-Identifier: BSD-2-Clause

# Converts a binary snippet/chunk file into a format able to be included into the main program

import sys

if len(sys.argv) != 3:
	sys.exit(1)

with open(sys.argv[1], "rb") as bin_file:
	with open(sys.argv[2], "wt") as inc_file:
		initialisers = []
		lengths = []
		locations = []
		byte_block = bin_file.read()
		pos = 0
		while True:
			prefix = byte_block[pos]
			if prefix == 0:
				pos += 1
				size = 256*byte_block[pos] + byte_block[pos + 1]
				pos += 2
				loc = 256*byte_block[pos] + byte_block[pos + 1]
				pos += 2
				block = byte_block[pos:pos + size]
				pos += size
				if not (loc == 0x0000 or loc == 0xFFE0 or loc == 0x0130 or loc == 0xFC00):
					print("Entry point for each snippet must be FFE0 and each chunk must be 0130. There is one block loaded at FC00 in addition.")
					sys.exit(2)
				if loc == 0xFFE0 or loc == 0x0130 or loc == 0xFC00:
					s = ", ".join([ f"0x{byte:02X}" for byte in block ])
					initialisers.append(s)
					lengths.append(size)
					locations.append(loc)
				elif loc == 0x0000:
					s = ''.join([ chr(ch) for ch in block])
					names = s.strip().split(' ')
				else:
					continue #ignore this entry
			else:
				break
		if len(initialisers) != len(names):
			print("Number of snippets and chunks does not match number of snippet and chunk names")
			sys.exit(4)
		for i in range(len(names)):
			if locations[i] == 0xFFE0:
				if lengths[i] > 16:
					print(f"Snippet may not exceed 16 bytes - snippet {names[i]} is {lengths[i]} bytes")
					sys.exit(3)
			if locations[i] == 0xFC00:
				if lengths[i] > 512:
					print(f"Debug chunk may not exceed 512 bytes - debug chunk {names[i]} is {lengths[i]} bytes")
					sys.exit(3)
		# Snippets
		inc_file.write("#if defined(SNIPPET_NAMES_INC)\n")
		inc_file.write("#define foreach_snippet(snippet) \\\n")
		for i in range(len(names)):
			if locations[i] == 0xFFE0:
				inc_file.write(f"\tsnippet({names[i]}) \\\n")
		inc_file.write(f"\tsnippet(SNIPPET_LEN)\n")
		inc_file.write("#endif // defined(SNIPPET_NAMES_INC)\n")
		inc_file.write("#if defined(SNIPPET_INITIALISERS_INC)\n")
		for i in range(len(names)):
			if locations[i] == 0xFFE0:
				inc_file.write(f"\t{{ {initialisers[i]} }},\n")
		inc_file.write("#endif // defined(SNIPPET_INITIALISERS_INC)\n")
		# Chunks
		inc_file.write("#if defined(CHUNK_NAMES_INC)\n")
		inc_file.write("#define foreach_chunk(chunk) \\\n")
		for i in range(len(names)):
			if locations[i] == 0x0130 or locations[i] == 0xFC00:
				inc_file.write(f"\tchunk({names[i]}) \\\n")
		inc_file.write(f"\tchunk(CHUNK_LEN)\n")
		inc_file.write("#endif // defined(CHUNK_NAMES_INC)\n")
		inc_file.write("#if defined(CHUNK_INITIALISERS_INC)\n")
		for i in range(len(names)):
			if locations[i] == 0x0130 or locations[i] == 0xFC00:
				inc_file.write(f"static const uint8_t chunk_{i}[] = {{ {initialisers[i]} }};\n")
		inc_file.write("static const uint chunk_len[CHUNK_LEN] = {\n")
		for i in range(len(names)):
			if locations[i] == 0x0130 or locations[i] == 0xFC00:
				inc_file.write(f"\t{lengths[i]},\n")
		inc_file.write("};\n")
		inc_file.write("static const uint8_t *(chunk_code[CHUNK_LEN]) = {\n")
		for i in range(len(names)):
			if locations[i] == 0x0130 or locations[i] == 0xFC00:
				inc_file.write(f"\tchunk_{i},\n")
		inc_file.write("};\n")
		inc_file.write("#endif // defined(CHUNK_INITIALISERS_INC)\n")
