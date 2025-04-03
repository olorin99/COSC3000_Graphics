include(FetchContent)


FetchContent_Declare(
        Ende
        GIT_REPOSITORY https://git.melamar.xyz/olorin99/Ende
        GIT_TAG        "master"
)

set(CANTA_BUILD_EXAMPLES  OFF CACHE BOOL "" FORCE)
set(CANTA_BUILD_TESTS  OFF CACHE BOOL "" FORCE)
set(CANTA_USE_SLANG  OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        Canta
        GIT_REPOSITORY  https://git.melamar.xyz/olorin99/Canta
        GIT_TAG         "main"
)

FetchContent_MakeAvailable(Ende Canta)