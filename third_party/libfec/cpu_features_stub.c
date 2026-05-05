/*
 * cpu_features_stub.c — replacement for cpu_features.s
 *
 * libfec 3.0.1's cpu_features.s is 32-bit x86-only assembly (uses pushl/popl
 * with general registers, which is invalid in 64-bit mode).  We replace it
 * with this stub.
 *
 * Returning 0 means "no SIMD features available", which causes
 * find_cpu_mode() in cpu_mode_x86.c to select the portable C backend.
 *
 * To re-enable SIMD later, replace this stub with a real implementation
 * that uses cpuid (e.g., __builtin_cpu_supports("sse2")) and returns the
 * appropriate libfec feature bitmask — see cpu_mode_x86.c for the bit
 * positions and the corresponding enum cpu_mode values.
 *
 * Part of the libfec vendoring; see README.UMTS-CHANGES.md.
 * License: LGPLv2.1 (matches the rest of libfec).
 */

unsigned long cpu_features(void)
{
	return 0;
}
