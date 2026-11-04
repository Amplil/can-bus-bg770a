/*
 * AtClient.hpp
 * Copyright (C) Seeed K.K.
 * MIT License
 */

#ifndef ATCLIENT_HPP
#define ATCLIENT_HPP

#include <functional>
#include <list>
#include <memory>
#include <string>
#include "../../internal/CountdownTimer.hpp"

namespace wiocellular
{
    namespace module
    {
        namespace at_client
        {

            /**
             * @~Japanese
             * @brief ATコマンドクライアント
             *
             * @tparam MODULE モジュールのクラス
             *
             * ATコマンドのクライアントのクラスです。
             */
            template <typename MODULE>
            class AtClient
            {
            public:
                using UrcHandlerType = bool(const std::string &);
                using UrcHandlerFunctionType = std::function<UrcHandlerType>;
                using PredicateType = bool(const std::string &);
                using PredFunctionType = std::function<PredicateType>;

            private:
                static constexpr char S3 = '\r';
                static constexpr char S4 = '\n';

            private:
                std::string Response_;
                std::list<UrcHandlerFunctionType> UrcHandlers_;

            public:
                /**
                 * @~Japanese
                 * @brief URCハンドラ
                 *
                 * URCハンドラのクラスです。
                 * URC処理ハンドラの登録/解除を自動的（RAII）に行います。
                 */
                class UrcHandler
                {
                private:
                    AtClient &AtClient_;
                    std::list<UrcHandlerFunctionType>::const_iterator It_;

                public:
                    /**
                     * @~Japanese
                     * @brief コンストラクタ
                     *
                     * @param [in] atClient ATコマンドクライアントのインスタンス。
                     * @param [in] handler URC処理ハンドラ。
                     *
                     * コンストラクタ。
                     */
                    UrcHandler(AtClient &atClient, const UrcHandlerFunctionType &handler)
                        : AtClient_{atClient}
                    {
                        It_ = AtClient_.registerUrcHandler(handler);
                    }

                    /**
                     * @~Japanese
                     * @brief デストラクタ
                     *
                     * デストラクタ。
                     */
                    ~UrcHandler()
                    {
                        AtClient_.unregisterUrcHandler(It_);
                    }

                    UrcHandler(UrcHandler &) = delete;
                    UrcHandler &operator=(UrcHandler &) = delete;
                };

            private:
                void writeCommand(const std::string &command)
                {
                    for (const auto c : command)
                    {
                        static_cast<MODULE &>(*this).getInterface().write(c);
                    }
                    static_cast<MODULE &>(*this).getInterface().write(S3);
                }

                bool processingUrc(const std::string &response)
                {
                    for (const auto &handler : UrcHandlers_)
                    {
                        if (handler(response))
                        {
                            return true;
                        }
                    }

                    return false;
                }

            public:
                /**
                 * @~Japanese
                 * @brief コンストラクタ
                 *
                 * コンストラクタ。
                 */
                AtClient() : Response_{},
                             UrcHandlers_{}
                {
                }

                /**
                 * @~Japanese
                 * @brief URC処理ハンドラを登録
                 *
                 * @param [in] handler URC処理ハンドラ。
                 * @return URC処理ハンドラのイテレータ。
                 *
                 * URC(unsolicited result code)を処理するハンドラを登録します。
                 * 戻り値のイテレータを使って、後でハンドラを解除することができます。
                 */
                std::list<UrcHandlerFunctionType>::const_iterator registerUrcHandler(const UrcHandlerFunctionType &handler)
                {
                    return UrcHandlers_.insert(UrcHandlers_.end(), handler);
                }

                /**
                 * @~Japanese
                 * @brief URC処理ハンドラを解除
                 *
                 * @param [in] it URC処理ハンドラのイテレータ。
                 *
                 * URC(unsolicited result code)を処理するハンドラを解除します。
                 */
                void unregisterUrcHandler(std::list<UrcHandlerFunctionType>::const_iterator it)
                {
                    UrcHandlers_.erase(it);
                }

                /**
                 * @~Japanese
                 * @brief URC処理ハンドラを登録
                 *
                 * @param [in] handler URC処理ハンドラ。
                 * @return URCハンドラのユニークポインタ。
                 *
                 * URC(unsolicited result code)を処理するハンドラを登録します。
                 * 戻り値を開放すると自動的にハンドラを解除します。
                 */
                std::unique_ptr<UrcHandler> registerUrcHandler2(const UrcHandlerFunctionType &handler)
                {
                    return std::make_unique<UrcHandler>(*this, handler);
                }

                /**
                 * @~Japanese
                 * @brief URC処理を実行
                 *
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 *
                 * レスポンスを確認して、URC(unsolicited result code)の処理を実行します。
                 * 永久にURC待ちしたいときはtimeoutに-1を指定します。
                 * URCを受信したときは、タイムアウト時間を待つことなく関数から返ります。
                 */
                void doWork(int timeout)
                {
                    const auto response = readResponse(timeout);
                    if (!response.empty())
                    {
                        static_cast<MODULE &>(*this).processingUrc(response);
                    }
                }

                /**
                 * @~Japanese
                 * @brief URC処理を実行
                 *
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 * @param [in] completion 完了確認のハンドラ。
                 * @retval true 完了確認のハンドラがtrueを返した
                 * @retval false タイムアウト
                 *
                 * レスポンスを確認して、URC(unsolicited result code)の処理を実行します。
                 * 永久にURC待ちしたいときはtimeoutに-1を指定します。
                 * URCを受信したときにcompletionを実行します。completionがtrueを返すと関数から返ります。
                 */
                bool doWork(int timeout, const std::function<bool()> &completion)
                {
                    assert(completion);

                    wiocellular::internal::CountdownTimer timer{timeout};
                    while (!completion())
                    {
                        if (timer.isTimeout())
                        {
                            return false;
                        }
                        doWork(timer.remaining());
                    }

                    return true;
                }

                /**
                 * @~Japanese
                 * @brief URC処理を実行（タイムアウトまで）
                 *
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 *
                 * レスポンスを確認して、URC(unsolicited result code)の処理を実行します。
                 * 永久にURC待ちしたいときはtimeoutに-1を指定します。
                 * URCを受信しても、タイムアウト時間まで関数から返りません。
                 */
                void doWorkUntil(int timeout)
                {
                    wiocellular::internal::CountdownTimer timer{timeout};
                    while (true)
                    {
                        if (timer.isTimeout())
                        {
                            return;
                        }
                        doWork(timer.remaining());
                    }
                }

                /**
                 * @~Japanese
                 * @brief コマンド書き込みと待機
                 *
                 * @param [in] command 送信コマンド。
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 *
                 * コマンドを書き込んでコマンドエコーを待機します。
                 * 待機する間、URC(unsolicited result code)のレスポンスがあれば対応する処理を実行します。
                 * 永久に待機したいときはtimeoutに-1を指定します。
                 */
                bool writeAndWaitCommand(const std::string &command, int timeout)
                {
                    writeCommand(command);

                    std::string commandWithS3{command};
                    commandWithS3.push_back(S3);
                    while (true)
                    {
                        const std::string response = readResponse(timeout, [&commandWithS3](const std::string &response) -> bool
                                                                  { return response == commandWithS3; });
                        if (response.empty())
                        {
                            return false;
                        }

                        if (response == commandWithS3)
                        {
                            return true;
                        }
                        else
                        {
                            static_cast<MODULE &>(*this).processingUrc(response);
                        }
                    }
                }

                /**
                 * @~Japanese
                 * @brief レスポンス読み込み
                 *
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 * @param [in] pred S4無しレスポンスのマッチング判定。
                 * @retval size()==0 受信データ無し
                 * @retval size()>0 受信データ
                 *
                 * 受信したレスポンスを読み込みます。
                 * レスポンスが無いときはサイズがゼロの文字列を返します。
                 * 永久に待機したいときはtimeoutに-1を指定します。
                 */
                std::string readResponse(int timeout, const PredFunctionType &pred = nullptr)
                {
                    wiocellular::internal::CountdownTimer timer{timeout};
                    while (true)
                    {
                        static_cast<MODULE &>(*this).getInterface().waitReadAvailable(timer.remaining());

                        while (true)
                        {
                            const auto c = static_cast<MODULE &>(*this).getInterface().read();
                            if (c < 0)
                            {
                                break;
                            }

                            if (0 <= c && c < 128)
                            {
                                switch (c)
                                {
                                case S4:
                                    if (!Response_.empty() && Response_.back() == S3)
                                    {
                                        Response_.pop_back();
                                    }
                                    if (!Response_.empty())
                                    {
                                        const auto value = Response_;
                                        Response_.clear();
                                        return value;
                                    }
                                    break;
                                default:
                                    if (c >= 32 || c == S3)
                                    {
                                        Response_.push_back(c);
                                        if (pred && pred(Response_))
                                        {
                                            const auto value = Response_;
                                            Response_.clear();
                                            return value;
                                        }
                                    }
                                    break;
                                }
                            }
                        }

                        if (timer.isTimeout())
                        {
                            return {};
                        }
                    }
                }

                /**
                 * @~Japanese
                 * @brief バイナリ書き込み
                 *
                 * @param [in] data 送信データ。
                 * @param [in] dataSize 送信データのサイズ。
                 *
                 * バイナリデータを書き込みます。
                 */
                void writeBinary(const void *data, size_t dataSize)
                {
                    assert(data != nullptr);
                    assert(dataSize >= 1);

                    for (size_t i = 0; i < dataSize; ++i)
                    {
                        static_cast<MODULE &>(*this).getInterface().write(static_cast<const uint8_t *>(data)[i]);
                    }
                }

                /**
                 * @~Japanese
                 * @brief バイナリ読み込み
                 *
                 * @param [in] data 受信データ。nullを指定すると読み捨てます。
                 * @param [in] dataSize 受信データのサイズ。
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 * @retval true 成功
                 * @retval false タイムアウト
                 *
                 * 受信したバイナリデータを読み込みます。
                 * 永久に待機したいときはtimeoutに-1を指定します。
                 */
                bool readBinary(void *data, size_t dataSize, int timeout)
                {
                    assert(dataSize >= 1);

                    wiocellular::internal::CountdownTimer timer{timeout};
                    size_t i = 0;
                    while (true)
                    {
                        static_cast<MODULE &>(*this).getInterface().waitReadAvailable(timer.remaining());

                        while (true)
                        {
                            const auto c = static_cast<MODULE &>(*this).getInterface().read();
                            if (c < 0)
                            {
                                break;
                            }

                            if (data)
                                static_cast<uint8_t *>(data)[i] = c;
                            if (++i >= dataSize)
                            {
                                return true;
                            }
                        }

                        if (timer.isTimeout())
                        {
                            return false;
                        }
                    }
                }

                /**
                 * @~Japanese
                 * @brief バイナリ読み込み（読み捨て）
                 *
                 * @param [in] dataSize 受信データのサイズ。
                 * @param [in] timeout タイムアウト時間[ミリ秒]。
                 * @retval true 成功
                 * @retval false タイムアウト
                 *
                 * 受信したバイナリデータを読み捨てます。
                 * 永久に待機したいときはtimeoutに-1を指定します。
                 */
                bool readBinaryDiscard(size_t dataSize, int timeout)
                {
                    assert(dataSize >= 1);

                    wiocellular::internal::CountdownTimer timer{timeout};
                    size_t i = 0;
                    while (true)
                    {
                        static_cast<MODULE &>(*this).getInterface().waitReadAvailable(timer.remaining());

                        while (true)
                        {
                            const auto c = static_cast<MODULE &>(*this).getInterface().read();
                            if (c < 0)
                            {
                                break;
                            }

                            if (++i >= dataSize)
                            {
                                return true;
                            }
                        }

                        if (timer.isTimeout())
                        {
                            return false;
                        }
                    }
                }
            };

        }
    }
}

#endif // ATCLIENT_HPP
