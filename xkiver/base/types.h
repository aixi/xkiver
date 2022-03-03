#ifndef XKIVER_BASE_TYPES_H
#define XKIVER_BASE_TYPES_H

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete;    \
    TypeName& operator=(const TypeName&) = delete

#define DISALLOW_MOVE_AND_ASSIGN(TypeName) \
    TypeName(TypeName&&) = delete;         \
    TypeName& operator=(TypeName&&) = delete

#define DISALLOW_COPY_MOVE_ASSIGN(TypeName) \
    TypeName(const TypeName&) = delete;    \
    TypeName& operator=(const TypeName&) = delete \
    TypeName(TypeName&&) = delete;         \
    TypeName& operator=(TypeName&&) = delete

#endif /* XKIVER_BASE_TYPES_H */
